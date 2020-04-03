
#include "point_cloud.h"

void make_proj_img_vec(
	PointCloud<PointXYZRGB>::Ptr pointcloud,
	vector<Mat> &proj_img_vec,
	vector<Mat> &depth_value_img_vec)
{
	for (int i = 0; i < total_num_cameras; i++)
	{
		Mat color_img(_height, _width, CV_8UC3, Scalar(0));

		Mat depth_value_img(_height, _width, CV_64F, -1);

		projection(pointcloud, i, color_img, depth_value_img);

		proj_img_vec[i] = color_img;

		depth_value_img_vec[i] = depth_value_img;
	}
}

map<int, list<PointXYZRGB>> make_3dGrid_map(
	PointCloud<PointXYZRGB>::Ptr registered_PC,
	vector<PointCloud<PointXYZRGB>::Ptr> pointclouds,
	vector<char> camera_order,
	int voxel_div_num) {

	vector<float> min(3), max(3);
	PointCloud<PointXYZRGB>::Ptr filtered_PC;

	filtered_PC = registered_PC;

	cout << "Before refinement, Point Cloud size:\t" << registered_PC->points.size() << endl;
	find_min_max(filtered_PC, min, max);

	float x_size = max[0] - min[0];
	float y_size = max[1] - min[1];
	float z_size = max[2] - min[2];
	int x_voxel_num = voxel_div_num;
	int y_voxel_num = voxel_div_num;
	int z_voxel_num = voxel_div_num;
	int total_voxel_num = x_voxel_num * y_voxel_num * z_voxel_num;

	map<int, list<PointXYZRGB>> m_cube;
	vector<char> voxel_index(total_voxel_num, -1);

	int x_voxel_index, y_voxel_index, z_voxel_index;
	int cube_index;
	int order = 0;
	while (order < camera_order.size()) {
		for (int point_idx = 0; point_idx < pointclouds[(int)camera_order[order]]->points.size(); point_idx++)
		{
			if (pointclouds[(int)camera_order[order]]->points[point_idx].x < min[0] ||
				pointclouds[(int)camera_order[order]]->points[point_idx].y < min[1] ||
				pointclouds[(int)camera_order[order]]->points[point_idx].z < min[2] ||
				pointclouds[(int)camera_order[order]]->points[point_idx].x > max[0] ||
				pointclouds[(int)camera_order[order]]->points[point_idx].y > max[1] ||
				pointclouds[(int)camera_order[order]]->points[point_idx].z > max[2])
				continue;

			x_voxel_index = (int)floor((pointclouds[(int)camera_order[order]]->points[point_idx].x - min[0]) / x_size * ((float)x_voxel_num - 1));
			y_voxel_index = (int)floor((pointclouds[(int)camera_order[order]]->points[point_idx].y - min[1]) / y_size * ((float)y_voxel_num - 1));
			z_voxel_index = (int)floor((pointclouds[(int)camera_order[order]]->points[point_idx].z - min[2]) / z_size * ((float)z_voxel_num - 1));
			cube_index = x_voxel_index * (y_voxel_num * z_voxel_num) + y_voxel_index * z_voxel_num + z_voxel_index;

			if ((int)voxel_index[cube_index] == -1) {
				PointXYZRGB _point;
				_point = pointclouds[(int)camera_order[order]]->points[point_idx];
				list<PointXYZRGB> PList;
				PList.push_back(_point);
				m_cube.insert(make_pair(cube_index, PList));
				voxel_index[cube_index] = camera_order[order];
			}

			else if (voxel_index[cube_index] == camera_order[order]) {
				PointXYZRGB _point;
				_point = pointclouds[(int)camera_order[order]]->points[point_idx];
				m_cube.find(cube_index)->second.push_back(_point);
			}
			else continue;
		}
		order++;
	}
	return m_cube;
}


Mat find_hole(
	Mat depthimg,
	int window_size)
{
	double avr_depth_value = 0;

	double ratio = 0.25;

	for (int i = 0; i < depthimg.rows; i++)
		for (int j = 0; j < depthimg.cols; j++)
			avr_depth_value += depthimg.at<double>(i, j);

	avr_depth_value = avr_depth_value / (_width * _height);

	Mat hole_image(_height, _width, CV_32F, -1);

	for (int rownum = 0; rownum < _height; rownum++)
	{
		for (int colnum = 0; colnum < _width; colnum++)
		{
			double center_pix_value;
			center_pix_value = depthimg.at<double>(rownum, colnum);

			if (center_pix_value == -1)
			{
				hole_image.at<float>(rownum, colnum) = 0;

				continue;
			}

			int different_pixel = 0;
			int out_of_range_pixel = 0;

			for (int h = rownum - window_size; h <= rownum + window_size; h++)
				for (int w = colnum - window_size; w <= colnum + window_size; w++)
				{
					if (h < 0 || w < 0 || h >= _height || w >= _width)
					{
						out_of_range_pixel++;
						continue;
					}

					if (abs(depthimg.at<double>(h, w) - center_pix_value) > avr_depth_value * ratio)
						different_pixel++;
				}



			if (different_pixel < ceil((pow((2 * window_size + 1), 2) - out_of_range_pixel) / 2))
				hole_image.at<float>(rownum, colnum) = 1;

			else
				hole_image.at<float>(rownum, colnum) = 0;
		}
	}

	return hole_image;
}

void hole_filling(
	vector<Mat> depthimgs,
	vector<Mat> colorimgs,
	vector<Mat> &filled_imgs)
{
	Mat hole_image;
	int window_size = 10;

	for (int num = 0; num < total_num_cameras; num++) {
		hole_image = find_hole(depthimgs[num], window_size);
		filled_imgs[num] = make_filled_image(depthimgs[num], colorimgs[num], hole_image, window_size);
	}
}

Mat make_filled_image(
	Mat depthimg,
	Mat colorimg,
	Mat hole_image,
	int window_size)
{
	Mat filled_image(_height, _width, CV_8UC3, Scalar::all(0));

	for (int rownum = 0; rownum < _height; rownum++)
	{
		for (int colnum = 0; colnum < _width; colnum++)
		{
			bool is_not_hole = hole_image.at<float>(rownum, colnum);

			if (is_not_hole)
			{
				filled_image.at<Vec3b>(rownum, colnum)[0] = colorimg.at<Vec3b>(rownum, colnum)[0];
				filled_image.at<Vec3b>(rownum, colnum)[1] = colorimg.at<Vec3b>(rownum, colnum)[1];
				filled_image.at<Vec3b>(rownum, colnum)[2] = colorimg.at<Vec3b>(rownum, colnum)[2];
				continue;
			}

			else
			{
				vector<ushort> pixel_sum(3);
				int pixel_count = 0;

				for (int h = rownum - window_size; h <= rownum + window_size; h++)
				{
					for (int w = colnum - window_size; w <= colnum + window_size; w++)
					{
						if (h < 0 || w < 0 || h >= _height || w >= _width) continue;

						else if (hole_image.at<float>(h, w) == 0) continue;

						else
						{
							pixel_sum[0] += colorimg.at<Vec3b>(h, w)[0];
							pixel_sum[1] += colorimg.at<Vec3b>(h, w)[1];
							pixel_sum[2] += colorimg.at<Vec3b>(h, w)[2];
							pixel_count++;
						}
					}
				}

				if (pixel_count == 0)
				{
					filled_image.at<Vec3b>(rownum, colnum)[0] = 255;
					filled_image.at<Vec3b>(rownum, colnum)[1] = 255;
					filled_image.at<Vec3b>(rownum, colnum)[2] = 255;
				}

				else
				{
					filled_image.at<Vec3b>(rownum, colnum)[0] = uchar(pixel_sum[0] / pixel_count);
					filled_image.at<Vec3b>(rownum, colnum)[1] = uchar(pixel_sum[1] / pixel_count);
					filled_image.at<Vec3b>(rownum, colnum)[2] = uchar(pixel_sum[2] / pixel_count);
				}
			}
		}
	}
	return filled_image;
}
