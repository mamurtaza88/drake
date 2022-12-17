# -*- mode: python -*-
# vi: set ft=python :

"""
Downloads and unpacks a MOSEK archive and makes its headers and
precompiled shared libraries available to be used as a C/C++
dependency.

Example:
    WORKSPACE:
        load("@drake//tools/workspace/mosek:repository.bzl", "mosek_repository")  # noqa
        mosek_repository(name = "foo")

    BUILD:
        cc_library(
            name = "foobar",
            deps = ["@foo//:mosek"],
            srcs = ["bar.cc"],
        )

Argument:
    name: A unique name for this rule.
"""

load("@drake//tools/workspace:execute.bzl", "which")

def _impl(repository_ctx):
    # When these values are updated, tools/dynamic_analysis/tsan.supp may also
    # need updating.
    mosek_major_version = 9
    mosek_minor_version = 2
    mosek_patch_version = 33

    if repository_ctx.os.name == "mac os x":
        mosek_platform = "osx64x86"
        sha256 = "72e3b46f1df67376f4a854db65841773fb4558bf8625395607bd1fa9ecf44879"  # noqa
    elif repository_ctx.os.name == "linux":
        mosek_platform = "linux64x86"
        sha256 = "e223389b9517fc636b75bb3cee817847df4bc9ca987ba279f40a9fbe5fa276d6"  # noqa
    else:
        fail(
            "Operating system is NOT supported",
            attr = repository_ctx.os.name,
        )

    # TODO(jwnimmer-tri) Port to use mirrors.bzl.
    template = "https://download.mosek.com/stable/{}.{}.{}/mosektools{}.tar.bz2"  # noqa
    url = template.format(
        mosek_major_version,
        mosek_minor_version,
        mosek_patch_version,
        mosek_platform,
    )
    root_path = repository_ctx.path("")
    strip_prefix = "mosek/{}.{}".format(
        mosek_major_version,
        mosek_minor_version,
    )

    repository_ctx.download_and_extract(
        url,
        root_path,
        sha256 = sha256,
        stripPrefix = strip_prefix,
    )

    platform_prefix = "tools/platform/{}".format(mosek_platform)

    if repository_ctx.os.name == "mac os x":
        install_name_tool = which(repository_ctx, "install_name_tool")

        # Note that libmosek64.dylib is (erroneously) a copy of
        # libmosek64.9.2.dylib instead of a symlink. Otherwise, the list of
        # files should include the following in place of bin/libmosek64.dylib:
        #
        # "bin/libmosek64.{}.{}.dylib".format(mosek_major_version,
        #                                     mosek_minor_version)
        files = [
            "bin/libcilkrts.5.dylib",
            "bin/libmosek64.dylib",
        ]

        for file in files:
            file_path = repository_ctx.path(
                "{}/{}".format(platform_prefix, file),
            )

            result = repository_ctx.execute([
                install_name_tool,
                "-id",
                file_path,
                file_path,
            ])

            if result.return_code != 0:
                fail(
                    "Could NOT change shared library identification name",
                    attr = result.stderr,
                )

        srcs = []

        bin_path = repository_ctx.path("{}/bin".format(platform_prefix))

        linkopts = [
            "-L{}".format(bin_path),
            "-lmosek64",
        ]
    else:
        files = [
            # N.B. We are using and installing MOSEK's copy of libcilkrts.so.5,
            # even though Ubuntu installs the same shared library by default on
            # all systems already. For some reason, Mosek fails when used with
            # Ubuntu's shared library. If Drake users have other third-party
            # code that assumes use of Ubuntu's libcilkrts, there could be
            # runtime conflicts; however, this risk seems low.
            "bin/libcilkrts.so.5",
            "bin/libmosek64.so.{}.{}".format(
                mosek_major_version,
                mosek_minor_version,
            ),
        ]

        linkopts = ["-pthread"]
        srcs = ["{}/{}".format(platform_prefix, file) for file in files]

    hdrs = ["{}/h/mosek.h".format(platform_prefix)]
    includes = ["{}/h".format(platform_prefix)]
    files = ["{}/{}".format(platform_prefix, file) for file in files]
    libraries_strip_prefix = ["{}/bin".format(platform_prefix)]

    file_content = """# -*- python -*-

# DO NOT EDIT: generated by mosek_repository()

load("@drake//tools/install:install.bzl", "install", "install_files")

licenses([
    "by_exception_only",  # MOSEK
    "notice",  # fplib AND Zlib
])

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "mosek",
    srcs = {},
    hdrs = {},
    includes = {},
    linkopts = {},
)

install_files(
    name = "install_libraries",
    dest = "lib",
    files = {},
    strip_prefix = {},
    visibility = ["//visibility:private"],
)

install(
   name = "install",
   docs = [
       "mosek-eula.pdf",
       "@drake//tools/workspace/mosek:LICENSE_CilkPlus",
   ],
   allowed_externals = [
       "@drake//tools/workspace/mosek:LICENSE_CilkPlus",
   ],
   deps = [":install_libraries"],
)
    """.format(srcs, hdrs, includes, linkopts, files, libraries_strip_prefix)

    repository_ctx.file(
        "BUILD.bazel",
        content = file_content,
        executable = False,
    )

mosek_repository = repository_rule(implementation = _impl)