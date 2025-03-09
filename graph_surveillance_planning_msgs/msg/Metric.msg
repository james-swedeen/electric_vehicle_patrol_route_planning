##
# @File: Metric.msg
#
# @brief
# Holds the value of a matrix through time.
#

# Name of the metric
string name
# The max value the metric can take or inf
float64 max_possible_value
# The min value the metric can take or -inf
float64 min_possible_value
# Indexed through time
float64[] values

