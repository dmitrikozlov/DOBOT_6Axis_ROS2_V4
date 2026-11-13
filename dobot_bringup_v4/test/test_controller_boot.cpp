// Unit tests for reboot detection (controller_boot.h): no controller needed.

#include <gtest/gtest.h>
#include <dobot_bringup/controller_boot.h>

TEST(ControllerBoot, NewUntilMarked)
{
    ControllerBoot boot;
    EXPECT_TRUE(boot.isNew(2214, 5000.0));
    boot.mark(2214, 5000.0);
    EXPECT_FALSE(boot.isNew(2214, 5000.0));
}

TEST(ControllerBoot, SameBootLaterIsNotNew)
{
    ControllerBoot boot;
    boot.mark(2214, 5000.0);
    // an hour on; the run time's whole seconds and the feed's age jitter it
    EXPECT_FALSE(boot.isNew(5814, 8600.9));
    EXPECT_FALSE(boot.isNew(5813, 8601.0));
}

TEST(ControllerBoot, RebootIsNew)
{
    ControllerBoot boot;
    boot.mark(2214, 5000.0);
    // power cycle at ~5100 s: up again 30 s later, the next stream at 5200 s
    EXPECT_TRUE(boot.isNew(70, 5200.0));
    // even once the new boot has run longer than the old one had
    EXPECT_TRUE(boot.isNew(3000, 8130.0));
    boot.mark(70, 5200.0);
    EXPECT_FALSE(boot.isNew(3000, 8130.0));
}
