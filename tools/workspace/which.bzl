# -*- python -*-

load("@drake//tools/workspace:execute.bzl", "path", "which")

def _impl(repository_ctx):
    command = repository_ctx.attr.command
    additional_paths = repository_ctx.attr.additional_search_paths
    found_command = which(repository_ctx, command, additional_paths)
    if found_command:
        repository_ctx.symlink(found_command, command)
    else:
        error_message = "Could not find {} on PATH={}".format(
            command,
            path(repository_ctx, additional_paths),
        )
        if repository_ctx.attr.allow_missing:
            repository_ctx.file(command, "\n".join([
                "#!/bin/sh",
                "echo 'ERROR: {}' 1>&2".format(error_message),
                "false",
            ]), executable = True)
        else:
            fail(error_message)
    build_file_content = """# -*- python -*-

# DO NOT EDIT: generated by which_repository()

# A symlink to {}.
exports_files(["{}"])

{}
""".format(found_command, command, repository_ctx.attr.build_epilog)
    repository_ctx.file(
        "BUILD.bazel",
        content = build_file_content,
        executable = False,
    )

which_repository = repository_rule(
    attrs = {
        "command": attr.string(mandatory = True),
        "additional_search_paths": attr.string_list(),
        "allow_missing": attr.bool(default = False),
        "build_epilog": attr.string(),
    },
    local = True,
    configure = True,
    implementation = _impl,
)

"""Alias the result of $(which $command) into a label @$name//:$command (or
@$command if name and command match). The PATH is set according to the path()
function in execute.bzl. The value of the user's PATH environment variable is
ignored.

Changes to any WORKSPACE or BUILD.bazel file will cause this rule to be
re-evaluated because it sets its local attribute. However, note that if neither
WORKSPACE nor **/BUILD.bazel change, then this rule will not be re-evaluated.
This means that adding or removing the presence of `command` on some entry in
the PATH (as defined above) will not be accounted for until something else
changes.

Args:
    command (:obj:`str`): Short name of command, e.g., "cat".
    additional_search_paths (:obj:`list` of :obj:`str`): List of additional
        search paths.
    allow_missing (:obj:`bool`): When True, errors will end up deferred to
        build time instead of fetch time -- a failure to find the command will
        still result in a BUILD.bazel target that provides the command, but
        the target will be missing.
    build_epilog: (Optional) Extra text to add to the generated BUILD.bazel.
"""