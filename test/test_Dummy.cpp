#include <boost/test/unit_test.hpp>
#include <kinematics_library/Dummy.hpp>

using namespace kinematics_library;

BOOST_AUTO_TEST_CASE(it_should_not_crash_when_welcome_is_called)
{
    kinematics_library::DummyClass dummy;
    dummy.welcome();
}
