'''
this script takes saved *.csv files from localization tests,
calculates errors with respect to ideal data and plots results
left plots - top is x,  middle is y and bottom is linear vels
right plots - top is angular error, middle is angular speed,
			bottom is 2d representation of path
'''

import matplotlib.pyplot as plt
import pandas as pd
import math

import os

class PlotConverter:
	def __init__(self, data_dir, save_dir, vels_dir):
		self.file_list = []
		self.save_dir = save_dir
		self.save_vels_dir = vels_dir
		self.data_dir = data_dir
		self.data_frame = pd.DataFrame()

	'''
	find all files with list of common name parts in previously specified data dir
	(for now it just adds one file by full name)
	'''
	def add_files_with_common_suffix(self, common_part):
		self.file_list.append(common_part)

	def find_files_in_specified_dir(self):
		file_list = os.listdir(self.data_dir)
		self.file_list = file_list
		self.file_list = [name.split('.')[0] for name in self.file_list]
		# for name in enumerate(self.file_list):
		# 	name = name.split('.')[0]


	def convert_and_display(self, file_name):
		file_path = os.path.join(self.data_dir, file_name+'.csv')
		self.data_frame = pd.read_csv(file_path)


		# remove 0 vels from beginning and end of file
		rushing_zeros, trailing_zeros = True, True
		while rushing_zeros:
			val = self.data_frame.head(1).index[0]
			if self.data_frame['vel_theta'][val] != 0.0 \
					or self.data_frame['vel_x'][val] != 0.0 \
				    or self.data_frame['vel_y'][val] != 0.0 :
				rushing_zeros = False
			else:
				self.data_frame.drop(val, inplace=True)

		while trailing_zeros:
			val = self.data_frame.tail(1).index[0]
			if self.data_frame['vel_theta'][val] != 0.0 \
					or self.data_frame['vel_x'][val] != 0.0 \
				    or self.data_frame['vel_y'][val] != 0.0 :
				trailing_zeros = False
			else:
				self.data_frame.drop(val, inplace=True)

		data_size = len(self.data_frame['ideal_x'])
		index_col = range(1, data_size + 1)
		self.data_frame['Index'] = index_col

		# fig, axes = plt.subplots(nrows=3, ncols=2)

		# errors in x axis
		# ax = axes[0,0]
		self.data_frame['error_x'] = self.data_frame['calculated_x'] \
		                             - self.data_frame['ideal_x']
		# self.data_frame.plot(label='error_x', kind='scatter',
		#                      x=0, y='error_x', color='black', ax=ax)
		#
		# ax.title.set_text('errors in x axis')
		# ax.set_ylabel('error in m')

		# save errors to file
		path_parent = os.path.dirname(self.data_dir)
		error_file = os.path.join(path_parent, self.save_dir, file_name + '-error-x.csv')
		self.data_frame.to_csv(error_file, columns=['Index', 'error_x'], index=False)



		# errors in y axis
		# ax = axes[1, 0]
		self.data_frame['error_y'] = self.data_frame['calculated_y'] \
		                             - self.data_frame['ideal_y']

		# self.data_frame.plot(label='error_y', kind='scatter',
		#                      x=0, y='error_y', color='black', ax=ax)
		# ax.title.set_text('errors in x axis')
		# ax.set_ylabel('error in m')

		# save errors to file
		path_parent = os.path.dirname(self.data_dir)
		error_file = os.path.join(path_parent, self.save_dir, file_name + '-error-y.csv')
		self.data_frame.to_csv(error_file, columns=['Index', 'error_y'], index=False)



		# errors in angle
		# ax = axes[0, 1]
		self.data_frame['error_theta'] = self.data_frame['calculated_theta'] \
		                                - self.data_frame['ideal_theta']
		# el_to_del = []
		# for idx, el in enumerate(self.data_frame['error_theta']):
		# 	if abs(el) > math.pi:
		# 		el_to_del.append(idx)
		#
		# self.data_frame.drop(el_to_del, inplace=True)
		# self.data_frame.plot(label='error_theta', kind='scatter',
		#                      x=0, y='error_theta', color='black', ax=ax)
		#
		# ax.title.set_text('errors in angle')
		# ax.set_ylabel('error in radians')

		# save errors to file
		path_parent = os.path.dirname(self.data_dir)
		error_file = os.path.join(path_parent, self.save_dir, file_name + '-error-theta.csv')
		self.data_frame.to_csv(error_file, columns=['Index', 'error_theta'], index=False)

		# dump vels to separate file
		path_parent = os.path.dirname(self.data_dir)
		error_file = os.path.join(path_parent, self.save_vels_dir, file_name + '-vels.csv')
		self.data_frame.to_csv(error_file, columns=['Index', 'vel_x', 'vel_y','vel_theta'], index=False)

		# dump position info to separate file
		path_parent = os.path.dirname(self.data_dir)
		position_file = os.path.join(path_parent,
		                             self.save_vels_dir,
		                             file_name + '-pos.csv')
		self.data_frame.to_csv(position_file,
		                       columns=['Index', 'ideal_x', 'ideal_y',
		                                'ideal_theta', 'calculated_x',
		                                'calculated_y', 'calculated_theta'],
		                       index=False)

		# plt.show()


	def calculate_errors(self):
		for file_name in self.file_list:
			self.convert_and_display(file_name)



if __name__ == '__main__':
	converter = PlotConverter(
		'/home/silver/INZYNIER/velma/ws_thesis/src/velma_experiments/data-raw',
		'error-data',
		'vels-data')

	# converter.add_files_with_common_suffix(['simple-amcl-circle'])
	converter.find_files_in_specified_dir()
	converter.calculate_errors()
	# converter.convert_and_display('simple-amcl-circle')