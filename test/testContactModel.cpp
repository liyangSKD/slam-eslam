#define BOOST_TEST_MODULE ContactModelTest
#include <boost/test/included/unit_test.hpp>

#include <eslam/ContactModel.hpp>
using namespace eslam;

struct FakeMLSAccess
{
    double const* z;
    double const* stddev;
    std::vector<base::Vector3d> points;

    FakeMLSAccess(double const* z, double const* stddev)
        : z(z), stddev(stddev) {}

    bool get(base::Vector3d const& pos, envire::MLSGrid::SurfacePatch& patch)
    {
        points.push_back(pos);

        int index = 0;
        if (pos.y() > 0)
            index += 2;
        if (pos.x() > 0)
            index++;

        patch.mean = z[index];
        patch.stdev = stddev[index];
        patch.height = 0;
        patch.horizontal = true;
        return true;
    }
};

void check_contact_point_selection(std::vector<ContactPoint> const& selected,
        BodyContactState const& state,
        double const* z,
        std::vector<int> expected)
{
    BOOST_CHECK_EQUAL(selected.size(), expected.size());
    for (unsigned int i = 0; i < expected.size(); ++i)
    {
        int expected_idx = expected[i];
        base::Vector3d p = state.points[expected_idx].position;
        p.z() = z[expected_idx];
        BOOST_CHECK_EQUAL(selected[i].point, p);
        BOOST_CHECK_SMALL((selected[i].point.z() - state.points[expected_idx].position.z()) - (z[i] - state.points[expected_idx].position.z()), 1e-6);
    }
}

BOOST_AUTO_TEST_CASE( test_eslam_passes_valid_global_position )
{
    BodyContactState state;
    state.time = base::Time::now();
    state.points.resize(4);
    state.points[0].position = base::Vector3d(1, 0, 0);
    state.points[0].contact  = 0.1;
    state.points[0].slip  = 0;
    state.points[0].groupId  = -1;
    state.points[1].position = base::Vector3d(-1, 0, 0);
    state.points[1].contact  = 0.1;
    state.points[1].slip  = 0;
    state.points[1].groupId  = -1;

    ContactModel model;
    model.setContactPoints(state, base::Quaterniond::Identity());
    {
        double z[4] = { 0, 0, 0, 0 };
        double stddev[4] = { 0, 0, 0, 0 };
        FakeMLSAccess access(z, stddev);
        base::Affine3d pose(Eigen::Translation3d(0.25, 0, 0));
        model.evaluatePose(pose, 1,
            boost::bind(&FakeMLSAccess::get, &access, _1, _2));

        // Check that the translation got applied. We assume that the contact
        // model processes the contact points in order, which it does so far
        BOOST_CHECK_SMALL( (access.points[0] - base::Vector3d(1.25, 0, 0)).norm(), 1e-6 );
        BOOST_CHECK_SMALL( (access.points[1] - base::Vector3d(-0.75, 0, 0)).norm(), 1e-6 );
    }
    {
        double z[4] = { 0, 0, 0, 0 };
        double stddev[4] = { 0, 0, 0, 0 };
        FakeMLSAccess access(z, stddev);
        base::Affine3d pose = Eigen::Translation3d(0.25, 0, 0) * Eigen::AngleAxisd(M_PI / 2, base::Vector3d::UnitZ());
        model.evaluatePose(pose, 1,
            boost::bind(&FakeMLSAccess::get, &access, _1, _2));

        // Check that the complete transform got applied. We assume that the
        // contact model processes the contact points in order, which it does so
        // far
        BOOST_CHECK_SMALL( (access.points[0] - base::Vector3d(.25, 1, 0)).norm(), 1e-6 );
        BOOST_CHECK_SMALL( (access.points[1] - base::Vector3d(.25, -1, 0)).norm(), 1e-6 );
    }
}

BOOST_AUTO_TEST_CASE( test_nogroup )
{
    BodyContactState state;
    state.time = base::Time::now();
    state.points.resize(4);
    state.points[0].position = base::Vector3d(-1, -1, 0);
    state.points[0].contact  = 0.1;
    state.points[0].slip  = 0;
    state.points[0].groupId  = -1;
    state.points[1].position = base::Vector3d(1, -1, 0);
    state.points[1].contact  = 0.1;
    state.points[1].slip  = 0;
    state.points[1].groupId  = -1;
    state.points[2].position = base::Vector3d(-1, 1, 0);
    state.points[2].contact  = 0.1;
    state.points[2].slip  = 0;
    state.points[2].groupId  = -1;
    state.points[3].position = base::Vector3d(1, 1, 0);
    state.points[3].contact  = 0.1;
    state.points[3].slip  = 0;
    state.points[3].groupId  = -1;

    ContactModel model;
    model.setContactPoints(state, base::Quaterniond::Identity());
    {
        double z[4] = { 0, 0, 0, 0 };
        double stddev[4] = { 1, 1, 1, 1 };
        FakeMLSAccess access(z, stddev);
        base::Affine3d pose(Eigen::Translation3d(0, 0, 0));
        BOOST_REQUIRE(model.evaluatePose(pose, 1,
            boost::bind(&FakeMLSAccess::get, &access, _1, _2)));

        std::vector<int> expected_points;
        expected_points.push_back(0);
        expected_points.push_back(1);
        expected_points.push_back(2);
        expected_points.push_back(3);
        check_contact_point_selection(model.getContactPoints(),
                state, z, expected_points);
        BOOST_CHECK_SMALL(model.getZDelta(), 1e-6);
        BOOST_CHECK_CLOSE(model.getZVar(), 0.5, 1e-6);
        BOOST_CHECK_CLOSE(model.getWeight(), 1, 1e-6);
    }
    {
        double z[4] = { 0, -0.12, -0.12, -0.12 };
        double stddev[4] = { 1, 1e9, 1e9, 1e9 };
        FakeMLSAccess access(z, stddev);
        base::Affine3d pose(Eigen::Translation3d(0, 0, 0));
        BOOST_REQUIRE(model.evaluatePose(pose, 1,
            boost::bind(&FakeMLSAccess::get, &access, _1, _2)));

        std::vector<int> expected_points;
        expected_points.push_back(0);
        expected_points.push_back(1);
        expected_points.push_back(2);
        expected_points.push_back(3);
        check_contact_point_selection(model.getContactPoints(),
                state, z, expected_points);
        BOOST_CHECK_SMALL(model.getZDelta(), 1e-6);
        BOOST_CHECK_CLOSE(model.getZVar(), 2, 1e-6);
        BOOST_CHECK_CLOSE(model.getWeight(), 1, 1e-6);
    }
}


BOOST_AUTO_TEST_CASE( test_group )
{
    BodyContactState state;
    state.time = base::Time::now();
    state.points.resize(4);
    state.points[0].position = base::Vector3d(-1, -1, 0.1);
    state.points[0].contact  = base::unknown<float>();
    state.points[0].slip  = 0;
    state.points[0].groupId  = 0;
    state.points[1].position = base::Vector3d(1, -1, -0.1);
    state.points[1].contact  = base::unknown<float>();
    state.points[1].slip  = 0;
    state.points[1].groupId  = 0;
    state.points[2].position = base::Vector3d(-1, 1, 0.1);
    state.points[2].contact  = base::unknown<float>();
    state.points[2].slip  = 0;
    state.points[2].groupId  = 1;
    state.points[3].position = base::Vector3d(1, 1, -0.1);
    state.points[3].contact  = base::unknown<float>();
    state.points[3].slip  = 0;
    state.points[3].groupId  = 1;

    ContactModel model;
    model.setContactPoints(state, base::Quaterniond::Identity());
    {
        double z[4] = { -0.1, -0.1, -0.1, -0.1 };
        double stddev[4] = { 1e9, 1, 1e9, 1 };
        FakeMLSAccess access(z, stddev);
        base::Affine3d pose(Eigen::Translation3d(0, 0, 0));

        BOOST_REQUIRE(model.evaluatePose(pose, 1,
            boost::bind(&FakeMLSAccess::get, &access, _1, _2)));

        std::vector<int> expected_points;
        expected_points.push_back(1);
        expected_points.push_back(3);
        check_contact_point_selection(model.getContactPoints(),
                state, z, expected_points);

        BOOST_CHECK_CLOSE(model.getZDelta(), -0, 1e-6);
        BOOST_CHECK_CLOSE(model.getZVar(), 1, 1e-6);
        BOOST_CHECK_CLOSE(model.getWeight(), 1, 1e-2);
    }
}


