FILE(REMOVE_RECURSE
  "CMakeFiles/sensor_models_generate_messages_py"
  "devel/lib/python2.7/dist-packages/sensor_models/srv/_Parameter_cmd.py"
  "devel/lib/python2.7/dist-packages/sensor_models/srv/_String_cmd.py"
  "devel/lib/python2.7/dist-packages/sensor_models/srv/_FingerIK_cmd.py"
  "devel/lib/python2.7/dist-packages/sensor_models/srv/__init__.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/sensor_models_generate_messages_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
