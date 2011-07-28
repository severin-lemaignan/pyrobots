FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/navigation_actionlib/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genaction_msgs"
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
  INCLUDE(CMakeFiles/ROSBUILD_genaction_msgs.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
