FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/navigation_actionlib/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/navigation_actionlib/NavigationAction.h"
  "../msg_gen/cpp/include/navigation_actionlib/NavigationGoal.h"
  "../msg_gen/cpp/include/navigation_actionlib/NavigationActionGoal.h"
  "../msg_gen/cpp/include/navigation_actionlib/NavigationResult.h"
  "../msg_gen/cpp/include/navigation_actionlib/NavigationActionResult.h"
  "../msg_gen/cpp/include/navigation_actionlib/NavigationFeedback.h"
  "../msg_gen/cpp/include/navigation_actionlib/NavigationActionFeedback.h"
  "../msg/NavigationAction.msg"
  "../msg/NavigationGoal.msg"
  "../msg/NavigationActionGoal.msg"
  "../msg/NavigationResult.msg"
  "../msg/NavigationActionResult.msg"
  "../msg/NavigationFeedback.msg"
  "../msg/NavigationActionFeedback.msg"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
