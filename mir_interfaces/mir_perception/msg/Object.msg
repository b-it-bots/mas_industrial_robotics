# The pose where the object can be found
geometry_msgs/PoseStamped pose

# The dimensions of the object
geometry_msgs/Vector3Stamped dimensions

# Name of the object determined e.g. by the recognition
string name

# Categorization of this object
string category

# The object's id in a database
int32 database_id

# Probability
float32 probability

# dominant object color
NamedColor dominant_color

# object shape
NamedShape shape

# View representation including an RGB image of the object, a point cloud,
# and potentially an embedding of the image. An array since we may have
# multiple views of the same object
ObjectView[] views

# The bounding box of the object
BoundingBox bounding_box

# The ROI of image (2D BBox)
sensor_msgs/RegionOfInterest roi
