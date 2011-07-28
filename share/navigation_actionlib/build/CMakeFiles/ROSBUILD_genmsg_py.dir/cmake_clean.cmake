FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/navigation_actionlib/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/navigation_actionlib/msg/__init__.py"
  "../src/navigation_actionlib/msg/_NavigationAction.py"
  "../src/navigation_actionlib/msg/_NavigationGoal.py"
  "../src/navigation_actionlib/msg/_NavigationActionGoal.py"
  "../src/navigation_actionlib/msg/_NavigationResult.py"
  "../src/navigation_actionlib/msg/_NavigationActionResult.py"
  "../src/navigation_actionlib/msg/_NavigationFeedback.py"
  "../src/navigation_actionlib/msg/_NavigationActionFeedback.py"
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
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
