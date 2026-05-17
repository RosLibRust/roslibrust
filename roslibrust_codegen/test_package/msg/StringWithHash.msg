# Test message for string constants containing # characters
# According to ROS spec, # inside string constant values should NOT be treated as comment

# This should have value "foo # bar"
string HASH_IN_VALUE=foo # bar

# This should have value "test#value"  
string NO_SPACES=test#value

# This should have value "#start"
string HASH_AT_START=#start

# This should have value "end#"
string HASH_AT_END=end#

# Regular field
string data
