#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/find_resource.h"
#include "drake/common/test_utilities/eigen_matrix_compare.h"
#include "drake/geometry/proximity/surface_mesh.h"
#include "drake/geometry/query_results/contact_surface.h"
#include "drake/geometry/scene_graph.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/hydroelastic_traction_calculator.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/framework/context.h"
#include "drake/systems/framework/diagram_builder.h"

namespace drake {

using geometry::ContactSurface;
using geometry::GeometryId;
using geometry::MeshFieldLinear;
using geometry::SceneGraph;
using geometry::SurfaceFaceIndex;
using geometry::SurfaceFace;
using geometry::SurfaceMesh;
using geometry::SurfaceVertex;
using geometry::SurfaceVertexIndex;
using math::RigidTransform;
using systems::Context;
using systems::Diagram;
using systems::DiagramBuilder;

namespace multibody {
namespace internal {

// Creates a surface mesh that covers the bottom of the "wetted surface",
// where the wetted surface is the part of the box that would be wet if the
// halfspace were a fluid. The entire wetted surface *would* yield
// an open box with five faces but, for simplicity, we'll only
// use the bottom face (two triangles).
std::unique_ptr<SurfaceMesh<double>> CreateSurfaceMesh() {
  std::vector<SurfaceVertex<double>> vertices;
  std::vector<SurfaceFace> faces;

  // Create the vertices, all of which are offset vectors defined in the
  // halfspace body frame.
  vertices.emplace_back(Vector3<double>(0.5, 0.5, -0.5));
  vertices.emplace_back(Vector3<double>(-0.5, 0.5, -0.5));
  vertices.emplace_back(Vector3<double>(-0.5, -0.5, -0.5));
  vertices.emplace_back(Vector3<double>(0.5, -0.5, -0.5));

  // Create the face comprising two triangles. The box penetrates into the
  // z = 0 plane from above. The contact surface should be constructed such that
  // the normals point out of geometry M and into geometry N. We assume that the
  // half space and box are geometries M and N, respectively. So, that means
  // the contact normals point downwards. We select windings for the triangle
  // vertices so that the normal will point in the [0, 0, -1] direction.
  //
  //             +z  +y
  //        v1 ___|__/____ v0
  //          /   | /    /
  //         /    |/    /
  //      --/----------/-- +x
  //       /     /|   /
  //   v2 /_____/_|__/ v3
  //           /  |
  faces.emplace_back(
      SurfaceVertexIndex(0), SurfaceVertexIndex(2), SurfaceVertexIndex(1));
  faces.emplace_back(
      SurfaceVertexIndex(2), SurfaceVertexIndex(0), SurfaceVertexIndex(3));

  auto mesh = std::make_unique<SurfaceMesh<double>>(
      std::move(faces), std::move(vertices));

  for (SurfaceFaceIndex f(0); f < mesh->num_faces(); ++f) {
    // Can't use an ASSERT_TRUE here because it interferes with the return
    // value.
    if (!CompareMatrices(mesh->face_normal(f), -Vector3<double>::UnitZ(),
        std::numeric_limits<double>::epsilon())) {
      throw std::logic_error("Malformed mesh; normals don't point downwards");
    }
  }

  return mesh;
}

GeometryId FindGeometry(
    const MultibodyPlant<double>& plant, const std::string body_name) {
  const auto& geometries = plant.GetCollisionGeometriesForBody(
      plant.GetBodyByName(body_name));
  DRAKE_DEMAND(geometries.size() == 1);
  return geometries[0];
}

// Creates a contact surface between the two given geometries.
std::unique_ptr<ContactSurface<double>> CreateContactSurface(
    GeometryId halfspace_id, GeometryId block_id,
    const math::RigidTransform<double>& X_WH) {
  // Create the surface mesh first (in the halfspace frame); we'll transform
  // it to the world frame *after* we use the vertices in the halfspace frame
  // to determine the hydroelastic pressure.
  auto mesh = CreateSurfaceMesh();

  // Create the "e" field values (i.e., "hydroelastic pressure") using
  // negated "z" values.
  std::vector<double> e_MN(mesh->num_vertices());
  for (SurfaceVertexIndex i(0); i < mesh->num_vertices(); ++i)
    e_MN[i] = -mesh->vertex(i).r_MV()[2];

  // Now transform the mesh to the world frame, as ContactSurface specifies.
  mesh->TransformVertices(X_WH);

  SurfaceMesh<double>* mesh_pointer = mesh.get();
  return std::make_unique<ContactSurface<double>>(
      halfspace_id, block_id, std::move(mesh),
      std::make_unique<MeshFieldLinear<double, SurfaceMesh<double>>>(
          "e_MN", std::move(e_MN), mesh_pointer));
}

// This fixture defines a contacting configuration between a box and a
// half-space in a local frame, Y. In Frame Y, half-space Frame H is
// positioned such that it passes through the origin and its normal Hz is
// aligned with Yz. In other words, the pose X_YH is the identity pose.
// The box pose X_YB is also set to be the identity pose and therefore the
// geometric center of the box is at the origin Yo, with the bottom (in frame Y)
// half of the box overlapping the half-space. The fixture unit-tests
// an arbitrary set of poses X_WY of frame Y in the world frame W to assess the
// frame invariance of the computed results, which only depend (modulo the
// "expressed-in" frame) on the relative pose of the bodies.
class MultibodyPlantHydroelasticTractionTests :
public ::testing::TestWithParam<RigidTransform<double>> {
 public:
  const HydroelasticTractionCalculator<double>& traction_calculator() const {
    return traction_calculator_;
  }

  const HydroelasticTractionCalculator<double>::Data& calculator_data() {
    return *calculator_data_;
  }

  // Sets all kinematic quantities (contact surface will be left untouched).
  // Note: see Data constructor for description of these parameters.
  void set_calculator_data(
      const RigidTransform<double>& X_WA, const RigidTransform<double>& X_WB,
      const SpatialVelocity<double>& V_WA,
      const SpatialVelocity<double>& V_WB) {
    calculator_data_ = std::make_unique<HydroelasticTractionCalculator<double>::
        Data>(
            X_WA, X_WB, V_WA, V_WB, &contact_surface());
  }

  const ContactSurface<double>& contact_surface() const {
      return *contact_surface_;
  }

  // Returns the default numerical tolerance.
  double tol() const { return tol_; }

  // Computes the spatial tractions due to the hydroelastic model. This computes
  // the spatial traction (in units of N/m??) at Vertex 0 of Triangle 0. We're
  // using the SpatialForce data type to hold these tractions, but they are
  // not forces. We'll use "Ft" rather than "F" here so we remember these are
  // tractions.
  void ComputeSpatialTractionsAtBodyOriginsFromHydroelasticModel(
      double dissipation, double mu_coulomb, SpatialForce<double>* Ft_Ao_W,
      SpatialForce<double>* Ft_Bo_W) {
    UpdateCalculatorData();

    // First compute the traction applied to Body A at point Q, expressed in the
    // world frame.
    HydroelasticQuadraturePointData<double> output =
        traction_calculator().CalcTractionAtPoint(
            calculator_data(), SurfaceFaceIndex(0),
            SurfaceMesh<double>::Barycentric(1.0, 0.0, 0.0), dissipation,
            mu_coulomb);

    // Compute the expected point of contact in the world frame. The class
    // definition and SetUp() note that the parameter to this test transforms
    // both bodies from their definition in Frame Y to the world frame.
    const RigidTransform<double>& X_WY = GetParam();
    const Vector3<double> p_YQ(0.5, 0.5, -0.5);
    const Vector3<double> p_WQ_expected = X_WY * p_YQ;

    // Verify the point of contact.
    for (int i = 0; i < 3; ++i)
      ASSERT_NEAR(output.p_WQ[i], p_WQ_expected[i], tol()) << i;

    const SpatialForce<double> Ft_Ac_W =
        traction_calculator().ComputeSpatialTractionAtAcFromTractionAtAq(
            calculator_data(), output.p_WQ, output.traction_Aq_W);

    // Shift to body origins. Traction on body B is equal and opposite.
    const Vector3<double>& p_WC = calculator_data().p_WC;
    const Vector3<double>& p_WAo = calculator_data().X_WA.translation();
    const Vector3<double>& p_WBo = calculator_data().X_WB.translation();
    const Vector3<double> p_CAo_W = p_WAo - p_WC;
    const Vector3<double> p_CBo_W = p_WBo - p_WC;
    *Ft_Ao_W = Ft_Ac_W.Shift(p_CAo_W);
    *Ft_Bo_W = -(Ft_Ac_W.Shift(p_CBo_W));
  }

  void SetBoxTranslationalVelocity(const Vector3<double>& v) {
    plant_->SetFreeBodySpatialVelocity(plant_context_,
        plant_->GetBodyByName("box"),
        SpatialVelocity<double>(Vector3<double>::Zero(), v));
  }

  void UpdateCalculatorData() {
    // Get the bodies that the two geometries are affixed to. We'll call these
    // A and B.
    const auto& query_object = plant_->get_geometry_query_input_port().
        template Eval<geometry::QueryObject<double>>(*plant_context_);
    const geometry::FrameId frameM_id = query_object.inspector().GetFrameId(
        contact_surface_->id_M());
    const geometry::FrameId frameN_id = query_object.inspector().GetFrameId(
        contact_surface_->id_N());
    const Body<double>& bodyA = *plant_->GetBodyFromFrameId(frameM_id);
    const Body<double>& bodyB = *plant_->GetBodyFromFrameId(frameN_id);

    // Get the poses of the two bodies in the world frame.
    const math::RigidTransform<double> X_WA =
        plant_->EvalBodyPoseInWorld(*plant_context_, bodyA);
    const math::RigidTransform<double> X_WB =
        plant_->EvalBodyPoseInWorld(*plant_context_, bodyB);

    // Get the spatial velocities for the two bodies (at the body frames).
    const SpatialVelocity<double> V_WA = plant_->EvalBodySpatialVelocityInWorld(
        *plant_context_, bodyA);
    const SpatialVelocity<double> V_WB = plant_->EvalBodySpatialVelocityInWorld(
        *plant_context_, bodyB);

    // (Re)-initialize the traction calculator data.
    set_calculator_data(X_WA, X_WB, V_WA, V_WB);
  }

  void ComputeSpatialForcesAtBodyOriginsFromHydroelasticModel(
      double dissipation, double mu_coulomb,
      SpatialForce<double>* F_Ao_W, SpatialForce<double>* F_Bo_W) {
    UpdateCalculatorData();

    SpatialForce<double> F_Ac_W;
    std::vector<HydroelasticQuadraturePointData<double>> quadrature_point_data;
    traction_calculator().ComputeSpatialForcesAtCentroidFromHydroelasticModel(
        calculator_data(), dissipation, mu_coulomb, &quadrature_point_data,
        &F_Ac_W);

    traction_calculator().ShiftSpatialForcesAtCentroidToBodyOrigins(
        calculator_data(), F_Ac_W, F_Ao_W, F_Bo_W);
  }

 private:
  void SetUp() override {
    // Read the two bodies into the plant.
    DiagramBuilder<double> builder;
    SceneGraph<double>* scene_graph;
    std::tie(plant_, scene_graph) = AddMultibodyPlantSceneGraph(&builder, 0.0);
    MultibodyPlant<double>& plant = *plant_;
    const std::string full_name = FindResourceOrThrow(
        "drake/multibody/plant/test/block_on_halfspace.sdf");
    Parser(&plant, scene_graph).AddModelFromFile(full_name);

    plant.Finalize();
    diagram_ = builder.Build();

    ASSERT_EQ(plant.num_velocities(), 12);
    ASSERT_EQ(plant.num_positions(), 14);

    // Create a context for this system.
    context_ = diagram_->CreateDefaultContext();
    plant_context_ =
        &diagram_->GetMutableSubsystemContext(plant, context_.get());

    // See class documentation for description of Frames Y, B, and H.
    const RigidTransform<double>& X_WY = GetParam();
    const auto& X_YH = RigidTransform<double>::Identity();
    const auto& X_YB = RigidTransform<double>::Identity();
    const RigidTransform<double> X_WH = X_WY * X_YH;
    const RigidTransform<double> X_WB = X_WY * X_YB;
    plant.SetFreeBodyPose(plant_context_, plant.GetBodyByName("ground"), X_WH);
    plant.SetFreeBodyPose(plant_context_, plant.GetBodyByName("box"), X_WB);

    GeometryId halfspace_id = internal::FindGeometry(plant, "ground");
    GeometryId block_id = internal::FindGeometry(plant, "box");
    contact_surface_ = CreateContactSurface(halfspace_id, block_id, X_WH);
  }

  const double tol_{10 * std::numeric_limits<double>::epsilon()};
  MultibodyPlant<double>* plant_;
  std::unique_ptr<Diagram<double>> diagram_;
  std::unique_ptr<Context<double>> context_;
  systems::Context<double>* plant_context_;
  std::unique_ptr<ContactSurface<double>> contact_surface_;
  HydroelasticTractionCalculator<double> traction_calculator_;
  std::unique_ptr<
      HydroelasticTractionCalculator<double>::Data> calculator_data_;
};

// Tests the traction calculation without any frictional or dissipation
// tractions.
TEST_P(MultibodyPlantHydroelasticTractionTests, VanillaTraction) {
  const double dissipation = 0.0;  // Units: s/m.
  const double mu_coulomb = 0.0;

  // Compute the spatial tractions at the origins of the body frames.
  multibody::SpatialForce<double> Ft_Ao_W, Ft_Bo_W;
  ComputeSpatialTractionsAtBodyOriginsFromHydroelasticModel(
      dissipation, mu_coulomb, &Ft_Ao_W, &Ft_Bo_W);

  // Re-express the spatial tractions in Y's frame for easy interpretability.
  // Note that f and tau here are still tractions, so our notation is being
  // misused a little here.
  const math::RotationMatrix<double>& R_WY = GetParam().rotation();
  const Vector3<double> f_Bo_Y = R_WY.transpose() * Ft_Bo_W.translational();
  const Vector3<double> tau_Bo_Y = R_WY.transpose() * Ft_Bo_W.rotational();

  // Check the spatial traction at p. We know that geometry M is the halfspace,
  // so we'll check the spatial traction for geometry N instead. Note that the
  // tangential components are zero.
  EXPECT_NEAR(f_Bo_Y[0], 0.0, tol());
  EXPECT_NEAR(f_Bo_Y[1], 0.0, tol());
  EXPECT_NEAR(f_Bo_Y[2], 0.5, tol());

  // A moment on the box will be generated due to the normal traction. The
  // origin of the box frame is located at (0,0,0) in the Y frame.
  // The moment arm at the point will be (.5, .5, -.5), again in the Y frame.
  // Crossing this vector with the traction at that point (0, 0, 0.5) yields the
  // following.
  EXPECT_NEAR(tau_Bo_Y[0], 0.25, tol());
  EXPECT_NEAR(tau_Bo_Y[1], -0.25, tol());
  EXPECT_NEAR(tau_Bo_Y[2], 0, tol());

  // The translational components of the two wrenches should be equal and
  // opposite.
  EXPECT_NEAR((Ft_Bo_W.translational() + Ft_Ao_W.translational()).norm(),
      0, tol());
}

// Tests the traction calculation with friction but without dissipation
// tractions.
TEST_P(MultibodyPlantHydroelasticTractionTests, TractionWithFriction) {
  const double dissipation = 0.0;  // Units: s/m.
  const double mu_coulomb = 1.0;

  // Give the box an initial (vertical) velocity along the +x axis in the Y
  // frame.
  const math::RotationMatrix<double>& R_WY = GetParam().rotation();
  SetBoxTranslationalVelocity(R_WY * Vector3<double>(1, 0, 0));

  // Compute the spatial tractions at the origins of the body frames.
  multibody::SpatialForce<double> Ft_Ao_W, Ft_Bo_W;
  ComputeSpatialTractionsAtBodyOriginsFromHydroelasticModel(
      dissipation, mu_coulomb, &Ft_Ao_W, &Ft_Bo_W);

  // Re-express the spatial tractions in Y's frame for easy interpretability.
  // Note that f and tau here are still tractions, so our notation is being
  // misused a little here.
  const Vector3<double> f_Bo_Y = R_WY.transpose() * Ft_Bo_W.translational();
  const Vector3<double> tau_Bo_Y = R_WY.transpose() * Ft_Bo_W.rotational();

  // Check the spatial traction at p. We know that geometry M is the halfspace,
  // so we'll check the spatial traction for geometry N instead. The coefficient
  // of friction is unity, so the total frictional traction will have
  // approximately the same magnitude as the normal traction. Note that the
  // regularized Coulomb friction model requires backing off of the tolerance
  // for comparing frictional traction components (against the Coulomb model);
  // the units are dissimilar (m/s vs. N and Nm) but using this as the tolerance
  // has proven useful.
  const double field_value = 0.5;  // in N/m??.
  const double regularization_scalar =
      traction_calculator().regularization_scalar();
  EXPECT_NEAR(f_Bo_Y[0], -mu_coulomb * field_value, regularization_scalar);
  EXPECT_NEAR(f_Bo_Y[1], 0.0, tol());
  EXPECT_NEAR(f_Bo_Y[2], field_value, tol());

  // A moment on the box will be generated due to the traction. The
  // origin of the box frame is located at (0,0,0) in the Y frame.
  // The moment arm at the point will be (.5, .5, -.5). Crossing this vector
  // with the traction at that point (-.5, 0, 0.5) yields the following.
  EXPECT_NEAR(tau_Bo_Y[0], 0.25, tol());
  EXPECT_NEAR(tau_Bo_Y[1], 0.0, regularization_scalar);
  EXPECT_NEAR(tau_Bo_Y[2], 0.25, regularization_scalar);

  // The translational components of the two wrenches should be equal and
  // opposite.
  EXPECT_NEAR((Ft_Bo_W.translational() + Ft_Ao_W.translational()).norm(),
      0, tol());
}

// Tests the traction calculation with dissipation tractions but without
// friction.
TEST_P(MultibodyPlantHydroelasticTractionTests, TractionWithDissipation) {
  const double dissipation = 1.0;  // Units: s/m.
  const double mu_coulomb = 0.0;

  // Give the box an initial (vertical) velocity along the -z axis in the Y
  // frame.
  const math::RotationMatrix<double>& R_WY = GetParam().rotation();
  const double separating_velocity = -1.0;
  SetBoxTranslationalVelocity(R_WY *
      Vector3<double>(0, 0, separating_velocity));

  // Compute the magnitude of the normal traction. Note that the damping
  // constant at each point will be field value * dissipation coefficient.
  const double field_value = 0.5;  // in N/m??.
  const double c = field_value * dissipation;  // N/m?? * s/m = Ns/m??.
  const double damping_traction_magnitude = c * -separating_velocity;  // N/m??.
  const double normal_traction_magnitude = field_value +
      damping_traction_magnitude;

  // Compute the spatial tractions at the origins of the body frames.
  multibody::SpatialForce<double> Ft_Ao_W, Ft_Bo_W;
  ComputeSpatialTractionsAtBodyOriginsFromHydroelasticModel(
      dissipation, mu_coulomb, &Ft_Ao_W, &Ft_Bo_W);

  // Re-express the spatial tractions in Y's frame for easy interpretability.
  // Note that f and tau here are still tractions, so our notation is being
  // misused a little here.
  const Vector3<double> f_Bo_Y = R_WY.transpose() * Ft_Bo_W.translational();
  const Vector3<double> tau_Bo_Y = R_WY.transpose() * Ft_Bo_W.rotational();

  // Check the spatial traction at p. We know that geometry M is the halfspace,
  // so we'll check the spatial traction for geometry N instead. The coefficient
  // of friction is unity, so the total frictional traction will have the same
  // magnitude as the normal traction.
  EXPECT_NEAR(f_Bo_Y[0], 0.0, tol());
  EXPECT_NEAR(f_Bo_Y[1], 0.0, tol());
  EXPECT_NEAR(f_Bo_Y[2], normal_traction_magnitude, tol());

  // A moment on the box will be generated due to the traction. The
  // origin of the box frame is located at (0,0,0) in the world frame.
  // The moment arm at the point will be (.5, .5, -.5). Crossing this vector
  // with the traction at that point (0, 0, normal_traction_magnitude) yields
  // the following.
  EXPECT_NEAR(tau_Bo_Y[0], 0.5 * normal_traction_magnitude, tol());
  EXPECT_NEAR(tau_Bo_Y[1], -0.5 * normal_traction_magnitude, tol());
  EXPECT_NEAR(tau_Bo_Y[2], 0.0, tol());

  // The translational components of the two wrenches should be equal and
  // opposite.
  EXPECT_NEAR((Ft_Bo_W.translational() + Ft_Ao_W.translational()).norm(),
      0, tol());
}

// Tests the traction calculation over an entire contact patch without
// friction or dissipation effecting the traction.
TEST_P(MultibodyPlantHydroelasticTractionTests, VanillaTractionOverPatch) {
  const double dissipation = 0.0;
  const double mu_coulomb = 0.0;

  // Compute the spatial forces at the origins of the body frames.
  multibody::SpatialForce<double> F_Ao_W, F_Bo_W;
  ComputeSpatialForcesAtBodyOriginsFromHydroelasticModel(
      dissipation, mu_coulomb, &F_Ao_W, &F_Bo_W);

  // Re-express the spatial forces in Y's frame for easy interpretability.
  const math::RotationMatrix<double>& R_WY = GetParam().rotation();
  const Vector3<double> f_Bo_Y = R_WY.transpose() * F_Bo_W.translational();
  const Vector3<double> tau_Bo_Y = R_WY.transpose() * F_Bo_W.rotational();

  // Check the spatial force. We know that geometry M is the halfspace,
  // so we'll check the spatial force for geometry N instead.
  const double field_value = 0.5;  // in N/m??.
  EXPECT_NEAR(f_Bo_Y[0], 0.0, tol());
  EXPECT_NEAR(f_Bo_Y[1], 0.0, tol());
  EXPECT_NEAR(f_Bo_Y[2], field_value, tol());

  // We expect no moment on the box.
  EXPECT_NEAR(tau_Bo_Y[0], 0.0, tol());
  EXPECT_NEAR(tau_Bo_Y[1], 0.0, tol());
  EXPECT_NEAR(tau_Bo_Y[2], 0.0, tol());
}

// Tests the traction calculation over an entire contact patch with
// friction but without dissipation effecting the traction.
TEST_P(MultibodyPlantHydroelasticTractionTests, FrictionalTractionOverPatch) {
  const double dissipation = 0.0;
  const double mu_coulomb = 1.0;

  // Give the box an initial (vertical) velocity along the +x axis in the Y
  // frame.
  const math::RotationMatrix<double>& R_WY = GetParam().rotation();
  SetBoxTranslationalVelocity(R_WY * Vector3<double>(1, 0, 0));

  // Compute the spatial forces at the origins of the body frames.
  multibody::SpatialForce<double> F_Ao_W, F_Bo_W;
  ComputeSpatialForcesAtBodyOriginsFromHydroelasticModel(
      dissipation, mu_coulomb, &F_Ao_W, &F_Bo_W);

  // Re-express the spatial forces in Y's frame for easy interpretability.
  const Vector3<double> f_Bo_Y = R_WY.transpose() * F_Bo_W.translational();
  const Vector3<double> tau_Bo_Y = R_WY.transpose() * F_Bo_W.rotational();

  // Check the spatial force. We know that geometry M is the halfspace,
  // so we'll check the spatial force for geometry N instead.  Note that the
  // regularized Coulomb friction model requires backing off of the tolerance
  // for comparing frictional traction components (against the Coulomb model);
  // the units are dissimilar (m/s vs. N and Nm) but using this as the tolerance
  // has proven useful.
  const double field_value = 0.5;  // in N/m??.
  const double regularization_scalar =
      traction_calculator().regularization_scalar();
  EXPECT_NEAR(f_Bo_Y[0], -mu_coulomb * field_value, regularization_scalar);
  EXPECT_NEAR(f_Bo_Y[1], 0.0, tol());
  EXPECT_NEAR(f_Bo_Y[2], field_value, tol());

  // A moment on the box will be generated due to the integral of the tractions.
  // The origin of the box frame is located at (0,0,0) in the Y frame.
  // The mean of all of the moment arms (we expect the tractions at each point
  // on the contact surface to be identical) will be (0, 0, -.5). Crossing this
  // vector with the traction at each point (-.5, 0, 0.5) yields (0, 0.25, 0).
  // The area of the contact surface is unity, so scaling this vector by the
  // area changes only the units, not the values.
  EXPECT_NEAR(tau_Bo_Y[0], 0.0, tol());
  EXPECT_NEAR(tau_Bo_Y[1], 0.25, regularization_scalar);
  EXPECT_NEAR(tau_Bo_Y[2], 0.0, tol());
}

// Tests the traction calculation over an entire contact patch without
// friction butwith dissipation effecting the traction.
TEST_P(MultibodyPlantHydroelasticTractionTests,
    TractionOverPatchWithDissipation) {
  const double dissipation = 1.0;  // Units: s/m.
  const double mu_coulomb = 0.0;

  // Give the box an initial (vertical) velocity along the -z axis in the Y
  // frame.
  const math::RotationMatrix<double>& R_WY = GetParam().rotation();
  const double separating_velocity = -1.0;
  SetBoxTranslationalVelocity(R_WY *
      Vector3<double>(0, 0, separating_velocity));

  // Compute the spatial forces at the origins of the body frames.
  multibody::SpatialForce<double> F_Ao_W, F_Bo_W;
  ComputeSpatialForcesAtBodyOriginsFromHydroelasticModel(
      dissipation, mu_coulomb, &F_Ao_W, &F_Bo_W);

  // Re-express the spatial forces in Y's frame for easy interpretability.
  const Vector3<double> f_Bo_Y = R_WY.transpose() * F_Bo_W.translational();
  const Vector3<double> tau_Bo_Y = R_WY.transpose() * F_Bo_W.rotational();

  // Compute the magnitude of the normal traction. Note that the damping
  // constant at each point will be field value * dissipation coefficient.
  const double field_value = 0.5;  // in N/m??.
  const double c = field_value * dissipation;  // N/m?? * s/m = Ns/m??.
  const double damping_traction_magnitude = c * -separating_velocity;  // N/m??.
  const double normal_traction_magnitude = field_value +
      damping_traction_magnitude;

  // Check the spatial force. We know that geometry M is the halfspace,
  // so we'll check the spatial force for geometry N instead.
  EXPECT_NEAR(f_Bo_Y[0], 0.0, tol());
  EXPECT_NEAR(f_Bo_Y[1], 0.0, tol());
  EXPECT_NEAR(f_Bo_Y[2], normal_traction_magnitude, tol());

  // We expect no moment on the box.
  EXPECT_NEAR(tau_Bo_Y[0], 0.0, tol());
  EXPECT_NEAR(tau_Bo_Y[1], 0.0, tol());
  EXPECT_NEAR(tau_Bo_Y[2], 0.0, tol());
}

// This fixture defines a contacting configuration between a box and a
// half-space in a local frame, Y. See MultibodyPlantHydroelasticTractionTests
// class documentation for a description of Frame Y.
class HydroelasticReportingTests :
public ::testing::TestWithParam<RigidTransform<double>> {
 public:
  const ContactSurface<double>& contact_surface() const {
      return *contact_surface_;
  }

  // Returns the default numerical tolerance.
  double tol() const { return tol_; }

  // Gets the expected pressure (in Pa) at Point Q in Frame Y. To get some
  // interesting values for testing, we define the pressure at Point Q using
  // a plane with normal in the direction [-1, -1, -1] that passes through the
  // origin.
  double pressure(const Vector3<double>& P_YQ) const {
    return pressure_field_normal().dot(P_YQ);
  }

  // Gets the normal to the pressure field in Frame Y.
  Vector3<double> pressure_field_normal() const {
    return Vector3<double>(-1, -1, -1).normalized();
  }

  const HydroelasticTractionCalculator<double>::Data& calculator_data() {
    return *calculator_data_;
  }

 private:
  void SetUp() override {
    // Set the poses in the Y frame. Identity poses should be fine for our
    // purposes because none of the reporting code itself relies upon these (the
    // code that does rely upon these, e.g., traction calculation, slip velocity
    // computation) is tested elsewhere in this file).
    const RigidTransform<double>& X_WY = GetParam();
    const RigidTransform<double> X_YA = RigidTransform<double>::Identity();
    const RigidTransform<double> X_YB = RigidTransform<double>::Identity();

    // Note: this test does not require valid GeometryIds.
    const GeometryId null_id;

    // Create the surface mesh first.
    auto mesh = CreateSurfaceMesh();

    // Create the e field values (i.e., "hydroelastic pressure").
    std::vector<double> e_MN(mesh->num_vertices());
    for (SurfaceVertexIndex i(0); i < mesh->num_vertices(); ++i)
      e_MN[i] = pressure(mesh->vertex(i).r_MV());

    SurfaceMesh<double>* mesh_pointer = mesh.get();
    contact_surface_ = std::make_unique<ContactSurface<double>>(
      null_id, null_id, std::move(mesh),
      std::make_unique<MeshFieldLinear<double, SurfaceMesh<double>>>(
          "e_MN", std::move(e_MN), mesh_pointer));

    // Set the velocities to correspond to one body fixed and one body
    // free so that we can test the slip velocity. Additionally, we'll
    // set the angular velocity to correspond to one of the bodies (A) spinning
    // along the normal to the contact surface. That will let us verify that the
    // velocity at the centroid of the contact surface is zero.
    const double spin_rate = 1.0;  // m/s.
    const SpatialVelocity<double> V_YA(
        Vector3<double>(0, 0, spin_rate), Vector3<double>::Zero());
    const SpatialVelocity<double> V_YB(
        Vector3<double>::Zero(), Vector3<double>::Zero());

    // Convert all quantities to the world frame.
    const RigidTransform<double> X_WA = X_WY * X_YA;
    const RigidTransform<double> X_WB = X_WY * X_YB;
    const SpatialVelocity<double> V_WA = X_WY.rotation() * V_YA;
    const SpatialVelocity<double> V_WB = X_WY.rotation() * V_YB;

    // Set the calculator data.
    calculator_data_ = std::make_unique<
        HydroelasticTractionCalculator<double>::Data>(
            X_WA, X_WB, V_WA, V_WB, contact_surface_.get());
  }

  const double tol_{10 * std::numeric_limits<double>::epsilon()};
  std::unique_ptr<ContactSurface<double>> contact_surface_;
  std::unique_ptr<HydroelasticTractionCalculator<double>::Data>
      calculator_data_;
};

// These transformations, denoted X_WY, are passed as parameters to the tests
// to allow changing the absolute (but not relative) poses of the two bodies.
const RigidTransform<double> poses[] = {
    RigidTransform<double>::Identity(),
    RigidTransform<double>(
        math::RotationMatrix<double>::MakeYRotation(M_PI_4),
        drake::Vector3<double>(1, 2, 3))
};

INSTANTIATE_TEST_SUITE_P(PoseInstantiations,
                        MultibodyPlantHydroelasticTractionTests,
                        ::testing::ValuesIn(poses));

// TODO(edrumwri) Break the tests below out into a separate file.

// Returns a distinct spatial force.
SpatialForce<double> MakeSpatialForce() {
  return SpatialForce<double>(Vector3<double>(1, 2, 3),
                              Vector3<double>(4, 5, 6));
}

// Returns a distinct vector (containing a single element) of quadrature point
// data.
std::vector<HydroelasticQuadraturePointData<double>> GetQuadraturePointData() {
  HydroelasticQuadraturePointData<double> data;
  data.p_WQ = Vector3<double>(3.0, 5.0, 7.0);
  data.face_index = SurfaceFaceIndex(1);
  data.vt_BqAq_W = Vector3<double>(11.0, 13.0, 17.0);
  data.traction_Aq_W = Vector3<double>(19.0, 23.0, 29.0);
  return { data };
}

HydroelasticContactInfo<double> CreateContactInfo(
    std::unique_ptr<ContactSurface<double>>* contact_surface,
    std::unique_ptr<HydroelasticContactInfo<double>>* contact_info) {
  // Create the contact surface using a duplicated arbitrary ID and identity
  // pose; pose and geometry IDs are irrelevant for this test.
  GeometryId arbitrary_id = GeometryId::get_new_id();
  *contact_surface = CreateContactSurface(arbitrary_id, arbitrary_id,
          RigidTransform<double>::Identity());

  // Create the HydroelasticContactInfo using particular spatial force and
  // quadrature point data.
  std::vector<HydroelasticQuadraturePointData<double>>
      quadrature_point_data = GetQuadraturePointData();
  return HydroelasticContactInfo<double>(contact_surface->get(),
                                         MakeSpatialForce(),
                                         std::move(quadrature_point_data));
}

// Verifies that the HydroelasticContactInfo structure uses the raw pointer
// and the unique pointer, as appropriate, on copy construction.
GTEST_TEST(HydroelasticContactInfo, CopyConstruction) {
  std::unique_ptr<ContactSurface<double>> contact_surface;
  std::unique_ptr<HydroelasticContactInfo<double>> contact_info;
  HydroelasticContactInfo<double> copy =
      CreateContactInfo(&contact_surface, &contact_info);

  // Verify that copy construction used the raw pointer.
  EXPECT_EQ(contact_surface.get(), &copy.contact_surface());

  // Copy it again and make sure that the surface is new.
  HydroelasticContactInfo<double> copy2 = copy;
  EXPECT_NE(contact_surface.get(), &copy2.contact_surface());

  // Verify that the spatial force was copied.
  EXPECT_EQ(copy.F_Ac_W().translational(), MakeSpatialForce().translational());
  EXPECT_EQ(copy.F_Ac_W().rotational(), MakeSpatialForce().rotational());

  // Verify that the quadrature point data was copied.
  EXPECT_EQ(copy.quadrature_point_data(), GetQuadraturePointData());
}

// Verifies that the HydroelasticContactInfo structure transfers ownership of
// the ContactSurface.
GTEST_TEST(HydroelasticContactInfo, MoveConstruction) {
  std::unique_ptr<ContactSurface<double>> contact_surface;
  std::unique_ptr<HydroelasticContactInfo<double>> contact_info;
  HydroelasticContactInfo<double> copy =
      CreateContactInfo(&contact_surface, &contact_info);
  HydroelasticContactInfo<double> moved_copy = std::move(copy);

  // Verify that the move construction retained the raw pointer.
  EXPECT_EQ(contact_surface.get(), &moved_copy.contact_surface());

  // Verify that the spatial force was copied.
  EXPECT_EQ(moved_copy.F_Ac_W().translational(),
            MakeSpatialForce().translational());
  EXPECT_EQ(moved_copy.F_Ac_W().rotational(), MakeSpatialForce().rotational());

  // Verify that the quadrature point data was copied.
  EXPECT_EQ(moved_copy.quadrature_point_data(), GetQuadraturePointData());
}

}  // namespace internal
}  // namespace multibody
}  // namespace drake

