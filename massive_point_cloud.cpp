#include "massive_point_cloud.h"

PointCloud<PointXYZRGBL>::Ptr make_PC_MPC(int camera, vector<Mat> color_imgs, vector<Mat> depth_imgs)
{
	PointCloud<PointXYZRGBL>::Ptr pointcloud(new PointCloud<PointXYZRGBL>);

	bool is_good_point = 0;

	Matrix4d RT_mat;
	RT_mat << m_CalibParams[camera].m_RotMatrix(0, 0), m_CalibParams[camera].m_RotMatrix(0, 1), m_CalibParams[camera].m_RotMatrix(0, 2), m_CalibParams[camera].m_Trans(0),
		m_CalibParams[camera].m_RotMatrix(1, 0), m_CalibParams[camera].m_RotMatrix(1, 1), m_CalibParams[camera].m_RotMatrix(1, 2), m_CalibParams[camera].m_Trans(1),
		m_CalibParams[camera].m_RotMatrix(2, 0), m_CalibParams[camera].m_RotMatrix(2, 1), m_CalibParams[camera].m_RotMatrix(2, 2), m_CalibParams[camera].m_Trans(2),
		0, 0, 0, 1;

	Matrix3d m_K_inv = m_CalibParams[camera].m_K.inverse();

	for (int y = 1; y < _height - 1; y++)
		for (int x = 1; x < _width - 1; x++)
		{
			Vec3b d = depth_imgs[camera].at<Vec3b>(y, x);
			double Z = denormalization_c(d[0], MinZ, MaxZ);
			double X = 0.0;
			double Y = 0.0;

			/*origin*/
			projection_UVZ_2_XY_PC(m_CalibParams[camera].m_ProjMatrix, x, y, Z, &X, &Y);

			PointXYZRGBL p;
			p.x = X;
			p.y = Y;
			p.z = -Z;

			Vec3b color = color_imgs[camera].at<Vec3b>(y, x);
			p.b = (float)color[0];
			p.g = (float)color[1];
			p.r = (float)color[2];
			p.a = camera;

			pointcloud->points.push_back(p);
		}

	return pointcloud;
}

vector<PointCloud<PointXYZRGBL>::Ptr> get_PC_of_every_camera_MPC(int frame, vector<vector<string>> color_names, vector<vector<string>> depth_names, vector<Mat> &color_imgs)
{
	vector<Mat> imgs(total_num_cameras);

	vector<Mat> imgs2(total_num_cameras);

	vector<PointCloud<PointXYZRGBL>::Ptr> pointclouds(total_num_cameras);

	for (int camera = 0; camera < total_num_cameras; camera++)
	{
		string folder_path = path + "/cam" + to_string(camera) + "/";
		Mat color_img = imread(folder_path + color_names[camera][frame]);
		imgs[camera] = color_img;
		Mat depth_img = imread(folder_path + depth_names[camera][frame]);
		imgs2[camera] = depth_img;
	}

	for (int camera = 0; camera < total_num_cameras; camera++)
	{
		PointCloud<PointXYZRGBL>::Ptr pointcloud(new PointCloud<PointXYZRGBL>);
		pointcloud = make_PC_MPC(camera, imgs, imgs2);
		pointclouds[camera] = pointcloud;
	}

	color_imgs = imgs;

	return pointclouds;
}

PointCloud<PointXYZRGBL>::Ptr make_registered_PC_MPC(vector<PointCloud<PointXYZRGBL>::Ptr> pointclouds)
{
	///* icp */
	//pcl::IterativeClosestPoint<pcl::PointXYZRGBL, pcl::PointXYZRGBL> icp;
	//icp.setInputSource(cloud_in);
	//icp.setInputTarget(cloud_out);
	//pcl::PointCloud<pcl::PointXYZRGBL> Final;
	//icp.align(Final);
	//std::cout << "has converged:" << icp.hasConverged() << " score: " <<
	//   icp.getFitnessScore() << std::endl;
	//std::cout << icp.getFinalTransformation() << std::endl;


	PointCloud<PointXYZRGBL>::Ptr registered_PC(new PointCloud<PointXYZRGBL>);

	for (int camera = 0; camera < total_num_cameras; camera++)
		for (int point_idx = 0; point_idx < pointclouds[camera]->points.size(); point_idx++)
			registered_PC->points.push_back(pointclouds[camera]->points[point_idx]);

	return registered_PC;
}

void view_PC_MPC(PointCloud<PointXYZRGBL>::Ptr pointcloud)
{
	int v1 = 0;

	PCLVisualizer viewer("PC viewer demo");
	viewer.setSize(1280, 1000);
	viewer.createViewPort(0.0, 0.0, 1.0, 1.0, v1);
	viewer.addCoordinateSystem(5.0);

	PointCloudColorHandlerRGBField<pcl::PointXYZRGBL > rgb_handler(pointcloud);
	viewer.addPointCloud(pointcloud, rgb_handler, "result", v1);
	while (!viewer.wasStopped()) viewer.spinOnce();
}

void projection_MPC(PointCloud<PointXYZRGBL>::Ptr pointcloud, int camera, Mat &img,	Mat &depthimg)
{
	PointCloud<PointXYZRGBL>::iterator cloudit;
	PointCloud<PointXYZRGBL>::iterator cloudend;

	double X;
	double Y;
	double Z;
	int u;
	int v;

	double dist;
	double w;

	for (cloudit = pointcloud->points.begin(), cloudend = pointcloud->points.end(); cloudit < cloudend; cloudit++) {
		if (cloudit->a == camera) {

			X = cloudit->x;
			Y = cloudit->y;
			Z = cloudit->z;

			Z = -Z;

			w = projection_XYZ_2_UV(
				m_CalibParams[camera].m_ProjMatrix,
				X,
				Y,
				Z,
				u,
				v);

			dist = find_point_dist(w, camera);

			if ((u < 0) || (v < 0) || (u > _width - 1) || (v > _height - 1)) continue;

			if (depthimg.at<double>(v, u) == -1)
				depthimg.at<double>(v, u) = dist;
			else
			{
				if (dist < depthimg.at<double>(v, u))
					depthimg.at<double>(v, u) = dist;

				else continue;
			}

			img.at<Vec3b>(v, u)[0] = uchar(cloudit->b);
			img.at<Vec3b>(v, u)[1] = uchar(cloudit->g);
			img.at<Vec3b>(v, u)[2] = uchar(cloudit->r);
		}
	}
}

