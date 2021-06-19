import tensorflow as tf
import numpy as np


class mobileNet:

	def __init__(self, model_path):

		self.graph = self.load_graph(model_path)
		self.input_layer = "Placeholder"
		self.output_layer = "final_result"
		self.dict = {0:"F20_20", 1:"M20", 2:"M20_100", 3:"M30", 4:"R20", 5:"S40_40"}

	def predict_one_label(self, image):

		# graph = self.load_graph(model_path)
		preprocessed_image = self.preprocess_image(image)

		input_name = "import/" + self.input_layer
		output_name = "import/" + self.output_layer
		input_operation = self.graph.get_operation_by_name(input_name)
		output_operation = self.graph.get_operation_by_name(output_name)

		with tf.Session(graph=self.graph) as sess:
			results = sess.run(output_operation.outputs[0],{
			    input_operation.outputs[0]: preprocessed_image
			})

		results = np.squeeze(results)
		highest_probability_index = np.argmax(results)
		top_k = results.argsort()[-5:][::-1]
		# labels = load_labels(label_file)
		# print ("predicted label ", self.dict[highest_probability_index])

		# for i in top_k:
		# 	print(self.dict[i], results[i])


		return self.dict[highest_probability_index]


	def load_graph(self, path):


		graph = tf.Graph()
		graph_def = tf.GraphDef()

		with open(path, "rb") as f:
			graph_def.ParseFromString(f.read())
		with graph.as_default():
			tf.import_graph_def(graph_def)

		return graph



	def preprocess_image(self, image):


		input_width=96
		input_height=96
		input_mean=0
		input_std=255
		# read the img from file
		# img_file = tf.read_file(image_path)

		# img_decoded = tf.image.decode_image(image, channels=3)

		float_caster = tf.cast(image, tf.float32)
		dims_expander = tf.expand_dims(float_caster, 0)
		# resize may be necessary for real time images 
		resized = tf.image.resize_bilinear(dims_expander, [input_width, input_height])
		normalized = tf.divide(tf.subtract(resized, [input_mean]), [input_std])

		sess = tf.Session()
		result = sess.run(normalized)

		return result