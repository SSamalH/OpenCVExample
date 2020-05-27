/******************************************************************************/
/*                                                                           */
/*  Description: Hirschvogel Piston Evaluation                                */
/*                                                                            */
/*  Version 0.0.1                                                             */
/*                                                                            */
/*  Copyright (C) 2020                                                        */
/*                                                                            */
/*  Feinwerktechnik Otto Harrandt GmbH                                        */
/*  Robert-Bosch-Strasse 25                                                   */
/*  71397 Leutenbach-Nellmersbach                                             */
/*  Germany                                                                   */
/*                                                                            */
/******************************************************************************/

#include <string>
#include <iostream>
#include <fstream>

#include <halali_base.h>
#include <halali_geometry.h>
#include <halali_raster.h>

#include "opencv2/opencv.hpp"
#include "cx_3d_common.h"
#include "cx_cam_common.h"

#include "piston_prepare.h"
#include "piston_raster.h"

#include <chrono>

/******************************************************************************/
/* DESCRIPTION: Parses command-line arguments, more specifically the input    */
/* file name (ASCII point cloud with one point per line, and x, y and z       */
/* coordinates being seperatated by a space) and the output .csv file.        */
/*                                                                            */
/* PARAMETERS:                                                                */
/*                                                                            */
/*  argc            [ I ]   Number of command-line arguments                  */
/*  argv            [ I ]   Command-line arguments as string array            */
/*  path_pcl_in     [ O ]   String pointer to input file path (.asc)          */
/*  path_csv_out    [ O ]   String pointer to output file path (.csv)         */
/*                                                                            */
/******************************************************************************/

void parse_cmd_line_args(int argc, char* argv[], std::string& path_in,
	std::string& path_out, int *label)
{
	if (argc == 5)
	{
		for (int i = 0; i < argc - 1; i++)
		{
			std::string arg = argv[i];
			if (arg.compare("-i") == 0)
			{
				path_in = argv[i + 1];
			}
			else if (arg.compare("-o") == 0)
			{
				path_out = argv[i + 1];
			}
			else if (arg.compare("-l") == 0)
			{
				*label = std::stoi(argv[i + 1]);
			}
		}

		//TODO robust detection of wrong inputs

	}
	else
	{
		std::cerr << "Execute as follows: hp_measurement.exe"
			<< "-i <Input Point Cloud> -o <Output CSV file>";
		exit(-1);
	}

	return;
}

inline void save_pts(AT::cx::c3d::PointCloud& pc, const char* filename)
{
	if (!pc.hasPoints())
	{
		return;
	}

	std::ofstream file_out(filename);

	unsigned int width = pc.points.width();
	unsigned int height = pc.points.height();

	for (unsigned int i = 0; i < height; i++)
	{
		for (unsigned int j = 0; j < width; j++)
		{
			file_out << pc.points.at<AT::cx::Point3f>(i, j).x << " "
				<< pc.points.at<AT::cx::Point3f>(i, j).y << " "
				<< pc.points.at<AT::cx::Point3f>(i, j).z << std::endl;
		}
	}

	file_out.close();
}

inline void save_pts(halali::PointCloud& pc, const char* filename)
{
	std::ofstream file_out(filename);

	//unsigned int width = pc.points.width();
	//unsigned int height = pc.points.height();

	for (unsigned int i = 0; i < pc.size(); i++)
	{
		file_out << pc.points[i].x << " " << pc.points[i].y << " " << pc.points[i].z << " " << pc.points[i].value << std::endl;
	}

	file_out.close();

	return;
}

inline double convert_y_coord(double y, double factor)
{
	return factor * y;
}

inline void copy_at2hp(AT::cx::c3d::PointCloud& at_pts,
	halali::PointCloud& pointcloud)
{
	/* Allocate correct size */
	unsigned int width = at_pts.points.width();
	unsigned int height = at_pts.points.height();

	pointcloud.resize(width * height);

	for (unsigned int i = 0; i < height; i++)
	{
		for (unsigned int j = 0; j < width; j++)
		{
			int idx = i * width + j;

			pointcloud.points[idx].x = at_pts.points.at<AT::cx::Point3f>(i, j).x;
			pointcloud.points[idx].y = convert_y_coord(at_pts.points.at<AT::cx::Point3f>(i, j).y, 0.0616); //0.15 is empirical value
			pointcloud.points[idx].z = at_pts.points.at<AT::cx::Point3f>(i, j).z;
		}
	}

	return;
}

inline void copy_at2hp(AT::cx::c3d::PointCloud& at_pts,
	halali::PointCloudOrganised& pointcloud, halali::PointCloudOrganised& normals)
{
	/* Allocate correct size */
	unsigned int width = at_pts.points.width();
	unsigned int height = at_pts.points.height();

	pointcloud.resize(width, height);
	normals.resize(width, height);

	for (unsigned int i = 0; i < height; i++)
	{
		for (unsigned int j = 0; j < width; j++)
		{
			//int idx = i * width + j;

			pointcloud.at(i,j).x = at_pts.points.at<AT::cx::Point3f>(i, j).x;
			pointcloud.at(i, j).y = convert_y_coord(at_pts.points.at<AT::cx::Point3f>(i, j).y, 0.0616); //0.15 is empirical value // 0.0616
			pointcloud.at(i, j).z = at_pts.points.at<AT::cx::Point3f>(i, j).z;

			normals.at(i, j).x
				= at_pts.normals.at<AT::cx::Point3f>(i, j).x;
			normals.at(i, j).y
				= at_pts.normals.at<AT::cx::Point3f>(i, j).z;
			normals.at(i, j).z
				= at_pts.normals.at<AT::cx::Point3f>(i, j).z;
		}
	}

	return;
}

inline void copy_at2pcl(AT::cx::c3d::PointCloud& at_pts,
	pcl::PointCloud<pcl::PointXYZ>& pcl_pts)
{
	/* Allocate correct size */
	unsigned int width = at_pts.points.width();
	unsigned int height = at_pts.points.height();


	pcl_pts.resize(width * height);

	for (unsigned int i = 0; i < height; i++)
	{
		for (unsigned int j = 0; j < width; j++)
		{
			int idx = i * width + j;

			pcl_pts.points[idx].x = at_pts.points.at<AT::cx::Point3f>(i, j).x;
			pcl_pts.points[idx].y = at_pts.points.at<AT::cx::Point3f>(i, j).y;
			pcl_pts.points[idx].z = at_pts.points.at<AT::cx::Point3f>(i, j).z;
		}
	}

	return;
}

inline void copy_at2pcl(AT::cx::c3d::PointCloud& at_pts,
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_pts,
	pcl::PointCloud<pcl::Normal>::Ptr normals)
{

	/* Allocate correct size */
	unsigned int width = at_pts.points.width();
	unsigned int height = at_pts.points.height();

	pcl_pts->resize(width * height);
	normals->resize(width * height);

	for (unsigned int i = 0; i < height; i++)
	{
		for (unsigned int j = 0; j < width; j++)
		{
			int idx = i * width + j;

			pcl_pts->points[idx].x = at_pts.points.at<AT::cx::Point3f>(i, j).x;
			pcl_pts->points[idx].y = at_pts.points.at<AT::cx::Point3f>(i, j).y;
			pcl_pts->points[idx].z = at_pts.points.at<AT::cx::Point3f>(i, j).z;

			normals->points[idx].normal_x
				= at_pts.normals.at<AT::cx::Point3f>(i, j).x;
			normals->points[idx].normal_y
				= at_pts.normals.at<AT::cx::Point3f>(i, j).z;
			normals->points[idx].normal_z
				= at_pts.normals.at<AT::cx::Point3f>(i, j).z;
		}
	}

	return;
}

inline void copy_at2hp(AT::cx::c3d::ZMap& zMap,
	halali::PointCloudOrganised& pointcloud)
{
	/* Allocate correct size */
	unsigned int width = zMap.img.width();
	unsigned int height = zMap.img.height();

	pointcloud.resize(width, height);

	for (unsigned int i = 0; i < height; i++)
	{
		for (unsigned int j = 0; j < width; j++)
		{
			//int idx = i * width + j;

			pointcloud.at(i, j).x = zMap.img.at<AT::cx::Point3f>(i, j).x;
			pointcloud.at(i, j).y = zMap.img.at<AT::cx::Point3f>(i, j).y; //0.15 is empirical value // 0.0616
			pointcloud.at(i, j).z = zMap.img.at<AT::cx::Point3f>(i, j).z;
		}

		std::cout << i <<  std::endl;

	}

	return;
}

inline void copy_at2hp(AT::cx::Image& range_img,
	halali::PointCloudOrganised& pointcloud)
{
	/* Allocate correct size */
	unsigned int width = range_img.width();
	unsigned int height = range_img.height();

	cv::Mat m = range_img.copyToMat();
	std::cout << m.cols << " " << m.rows << std::endl;

	pointcloud.resize(width, height);

	for (unsigned int i = 0; i < height; i++)
	{
		for (unsigned int j = 0; j < width; j++)
		{
			//int idx = i * width + j;

			pointcloud.at(i, j).x = j;
			pointcloud.at(i, j).y = i; 
			pointcloud.at(i, j).z = range_img.at<unsigned int>(i, j);
			//std::cout << range_img.at<double>(i,j) << std::endl;
		}

		std::cout << i << std::endl;

	}

	return;
}

inline void rangeimage2pointcloud(AT::cx::Image& range_image,
	halali::PointCloudOrganised& pointcloud)
{
	int width = range_image.width();
	int height = range_image.height();

	pointcloud.resize(width, height);

	// TODO no idea why AT image is divided like that
	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width / 2; j++)
		{
			pointcloud.at(i, 2 * j).x = 2 * j;
			pointcloud.at(i, 2 * j).y = i;
			pointcloud.at(i, 2 * j).z = range_image.at<unsigned int>(i, j) % 65536;

			pointcloud.at(i, 2 * j + 1).x = 2 * j + 1;
			pointcloud.at(i, 2 * j + 1).y = i;
			pointcloud.at(i, 2 * j + 1).z = range_image.at<unsigned int>(i, j + width / 2) % 65536;

		}
	}

	return;
}

void fetch_AT_point_cloud(halali::PointCloudOrganised& pointcloud)
{
	try
	{
		// Find camera
		std::string device_uri = AT::cx::discoverAndChooseDevice()->deviceURI;
		std::cout << "Open Device: " << device_uri.c_str() << std::endl;

		// Connect to camera
		auto cam = AT::cx::DeviceFactory::openDevice(device_uri);

		AT::cx::Variant value;
		cam->getParam("DeviceScanType", value);
		if (std::string(value) != "Linescan3D")
		{
			std::cerr << "Current sensor devicemode is not 'Linescan3D', we activate it..." << std::endl;
			cam->setParam("DeviceScanType", "Linescan3D");
		}

		// Load calibration file from sensor
		// TODO define calib, import from there
		AT::cx::c3d::Calib calib;
		AT::cx::downloadCalib(cam->getHandle(), calib);
		//std::string basePath = "C:\\Users\\akoehn\\Documents\\Projekte\\ATOP_Hairpin\\AT_Festo_Achse\\";
		//std::string calib_fname = basePath + "calibration.xml";
		//calib.load(calib_fname);

		// 3. set relevant acquisition parameters in calibration, we assume we have only one aoi active
		AT::cx::updateCalib(cam->getHandle(), calib, 1);

		// Step size between profiles
		//double dy = 0.03;
		double dy = 1.0;
		calib.set(CX_3D_PARAM_SY, dy);

		// print updated calibration that is used to compute the point cloud
		AT::cx::printInfo(calib.getHandle());

		// set the invalid value that will bes used in point cloud for all rangemap values that are zero 
		float ivd = NAN;	// could be any number e.g. -10.0f, if NAN is used the points are automatically suppressed in showPointCloud 
		calib.set(CX_3D_PARAM_METRIC_IDV, ivd);

		// Trigger cache update. When calling with argument > 0, the internal cache gets updated. 
		// This prevents a cache update at first call to metric functions cx_3d_range2calibratedABC.
		calib.set(CX_3D_PARAM_METRIC_CACHE_MODE, int(1));

		// 4. Allocate and queue internal acquisition buffers
		cam->allocAndQueueBuffers();

		// 5. Start image acquisition
		cam->startAcquisition();

		// 6. Grab acquisition buffer, wait for valid buffer with optional timeout. Timeout is given in ms.
		AT::cx::DeviceBuffer buffer = cam->waitForBuffer(10000);

		// 7. get image data from buffer and do some processing on the image data (or get a copy for later use)
		// \note img holds a reference to the image data in the DeviceBuffer, if you need the image data after cx_queueBuffer you need to clone the image!
		{
			auto rangeImg = buffer.getImage();

			rangeImg->save("range_image.tif");

			copy_at2hp(*rangeImg, pointcloud);

			// 8. calculate point cloud
			//AT::cx::c3d::PointCloud pc(rangeImg->height(), rangeImg->width());

			// 4.1 set scaling for rectified image: x = -30...+30, y = 0...100, z = -10...30, dx=600, dy=1000, dz=4000
			unsigned zMap_dx = 600;		// with 60mm object-dx  we get 0.01 mm X-resolution
			unsigned zMap_dy = 1000;	// with 100mm object-dy we get 0.01 mm Y-resolution
			unsigned zMap_dz = 4000;	// with 40mm object-dz  we get 0.001 mm Z-resolution
			float obj_dx = 60.0f;
			float obj_dy = 100.0f;
			float obj_dz = 40.0f;
			AT::cx::Point3f offset(-obj_dx / 2.0f, 0.0f, -10.0f);
			AT::cx::Point3f scale(obj_dx / float(zMap_dx), obj_dy / float(zMap_dy), obj_dz / float(zMap_dz));

			// 4.2 calculate the rectified image (Z-Map) 
			// possible pixel formats: CX_PF_COORD3D_C32f, CX_PF_COORD3D_C16
			// Note: We always ignore range values of zero, also when flag CX_3D_METRIC_MARK_Z_INVALID_DATA is not set
			AT::cx::c3d::ZMap zMap(zMap_dy, zMap_dx, CX_PF_COORD3D_C32f, scale, offset);
			auto t1 = std::chrono::high_resolution_clock::now();
			AT::cx::c3d::calculateZMap(calib, *rangeImg, zMap, CX_3D_METRIC_MARK_Z_INVALID_DATA | CX_3D_METRIC_INTERP_IDW);
			//AT::cx::c3d::calculatePointCloud(calib, *rangeImg, pc, CX_PF_COORD3D_ABC32f);

			// 9. show point cloud using OpenCV Viz3d module
			//pc.computeNormals();											// compute normals from point cloud points
			//AT::cx::normalizeMinMax8U(*rangeImg, pc.colors);					// compute colors from height values of range map. Function defined in cx_3d_common.

			// 10. optionally save PointCloud to file
			//save_pts(pc, "tmp_pts.asc");
		}

		// 11. Queue back the buffer to the devices acquisition engine.
		// \note From now on any references to the buffer images are not valid anymore and might be overwritten with new image data at any time!
		buffer.queueBuffer();

		// 12. Stop acquisition
		cam->stopAcquisition();

		// 13. Cleanup
		cam->freeBuffers();
		cam->close();
	}
	catch (std::exception & e)
	{
		std::cout << "exception caught, msg:" << e.what();
		exit(-3);
	}
	return;
}

int main(int argc, char* argv[])
{

	halali::PointCloudOrganised pointcloud;
	halali::Image<int> mask;
	halali::Image<double> f, fx, fy;
	int label;

	//fetch_AT_point_cloud(pointcloud);

	AT::cx::Image range_image;
	std::string filename_in, filename_out;
	char* filename_out_c;

	parse_cmd_line_args(argc, argv, filename_in, filename_out, &label);
	filename_out_c = (char*)malloc((filename_out.size() + 1) * sizeof(char));
	strcpy(filename_out_c, filename_out.c_str());

	range_image.load(filename_in);

	std::cout << "Beginning transformation... ";
	rangeimage2pointcloud(range_image, pointcloud);
	std::cout << "Done." << std::endl;

	int width = pointcloud.width();
	int height = pointcloud.height();

	std::cout << "Preparing mask... ";

	mask.resize(width, height);
	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			if (pointcloud.at(i, j).z < 1e-3)
			{
				mask.at(i, j) = 255;
			}
			else
			{
				mask.at(i, j) = 0;
			}
		}
	}
	std::cout << "Done." << std::endl;

	std::cout << "Inpainting... ";
	//halali::inpaint(pointcloud, mask);
	halali::prepare_mask(pointcloud, mask, 0.0);
	halali::fill_holes(pointcloud, mask);
	std::cout << "Done." << std::endl;

	//TODO removal of no data values?

	/* Converting back to depth image */
	std::cout << "Creating image... ";
	f.resize(width, height);
	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			f.at(i, j) = pointcloud.at(i, j).z;

			//std::cout << pointcloud.at(i, j).z << std::endl;
		}
	}

	std::cout << "Done." << std::endl;


	double sigma = 3.0;
	double accuracy = 2.0;

	std::cout << "Convolve image... ";
	f.gaussian_blur(sigma, accuracy);
	//f.derivative_x();
	std::cout << "Done." << std::endl;

	fx.resize(width, height);
	fy.resize(width, height);
	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			fx.at(i, j) = f.at(i, j);
			fy.at(i, j) = f.at(i, j);
		}
	}

	std::cout << "Compute derivatives... ";
	fx.derivative_x();
	std::cout << "dx done... ";
	fy.derivative_y();
	std::cout << "dy done." << std::endl;

	// Define ROI
	int min_x = 512;
	int max_x = 1040;
	int min_y = 0;
	int max_y = 3088;
	int roi_width = max_x - min_x;
	int roi_height = max_y - min_y;

	// Compute gradient angles and magnitude
	int hog_cell_size = 16; // Do not change that as we assume that image size is a multiple of hog_cell_size (no mirroring right now)
	int cells_x, cells_y;
	int angle_bin_n = 8;
	halali::Image<double> angles(roi_width, roi_height);
	halali::Image<double> magnitude(roi_width, roi_height);

	for (int i = 0; i < roi_height; i++)
	{
		for (int j = 0; j < roi_width; j++)
		{
			double dx = fx.at(i+min_y, j+min_x);
			double dy = fy.at(i+min_y, j+min_x);
			
			/* Compute gradient mangitude */
			magnitude.at(i, j) = sqrt(dx * dx + dy * dy);
			
			/* Compute gradient angle and make sure it is positive */
			double a = atan2(dy, dx);
			a = a > 0 ? a : 2 * M_PI + a;

			angles.at(i, j) = a;
		}
	}

	/* Compute HOGs */
	cells_x = (int)floor((double)roi_width / (double)hog_cell_size);
	cells_y = (int)floor((double)roi_height / (double)hog_cell_size);

	std::vector<double> hog(cells_x * cells_y * angle_bin_n, 0.0);

	double bin_width = (double)angle_bin_n / (2.0 * M_PI);

	for (int i = 0; i < roi_height; i++)
	{
		int c_y = (int)i / hog_cell_size;

		for (int j = 0; j < roi_width; j++)
		{
			int c_x = (int)j / hog_cell_size;
			int idx = (int)floor(angles.at(i, j) / bin_width);

			hog[c_y * cells_x * angle_bin_n + c_x * angle_bin_n + idx] += magnitude.at(i, j);
		}
	}

	/* Export HOGs such that they can be used by LIBSVM */
	std::ofstream file;
	file.open(filename_out, std::ofstream::app);

	if (label == 0)
	{
		file << "-1 ";
	}
	else
	{
		file << "+1 ";
	}

	for (int i = 0; i <hog.size();i++)
	{
		file << i + 1 << ":" << hog[i] << " ";
	}
	file << std::endl;

	file.close();
	

	return 0;
}


/*int main(int argc, char* argv[])
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud;
	//pcl::PointCloud<pcl::Normal>::Ptr pcl_normals;
	halali::PointCloud pointcloud;
	//std::vector<halali::Vector3f> normals;
	halali::VectorField normals;

	//fetch_AT_point_cloud(pointcloud, normals);
	//pointcloud.export_to_pcd("pointcloud.pcd");
	pointcloud.import_from_pcd("pointcloud.pcd");

	std::cout << pointcloud.width() << "x" << pointcloud.height() << std::endl;

	pointcloud.find_NaN(100.0);

	pointcloud.export_to_ascii("test.asc");

	exit(-9);

	auto start = std::chrono::steady_clock::now();
	halali::laplacian(pointcloud, 1.0, 2.0);
	halali::compute_normals_exp(pointcloud, 1.0, 2.0, normals);
	auto end = std::chrono::steady_clock::now();
	std::cout << "Computing normals (" << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() << " microseconds)." << std::endl;

	start = std::chrono::steady_clock::now();
	//halali::vector_field_smoothing(normals, 1.0, 2.0);
	end = std::chrono::steady_clock::now();
	std::cout << "Smoothing the normals (" << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() << " microseconds)." << std::endl;

	start = std::chrono::steady_clock::now();
	piston::compute_curvature(pointcloud, normals);
	end = std::chrono::steady_clock::now();
	std::cout << "Compute curvature (" << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() << " microseconds)." << std::endl;



	pointcloud.export_to_ascii("pointcloud_test.asc");

	//save_pts(pointcloud, "pointcloud.asc");
	//piston::rasterize(pointcloud, 5.0);

	return 0;
}

int main5(int argc, char* argv[])
{

	halali::PointCloud pointcloud;
	halali::Image<int> mask;

	//fetch_AT_point_cloud(pointcloud);

	AT::cx::Image range_image;
	std::string filename_in, filename_out;
	char *filename_out_c;
	int label;

	parse_cmd_line_args(argc, argv, filename_in, filename_out, &label);
	filename_out_c = (char*)malloc((filename_out.size() + 1) * sizeof(char));
	strcpy(filename_out_c, filename_out.c_str());

	range_image.load(filename_in);

	std::cout << "Beginning transformation... ";
	rangeimage2pointcloud(range_image, pointcloud);
	std::cout << "Done." << std::endl;

	//pointcloud.export_to_ascii("Pointcloud_before_inpainted.asc");

	//pointcloud.resize(1280, 300);

	//pointcloud.export_to_ascii("pts.asc");

	int width = pointcloud.width();
	int height = pointcloud.height();

	std::cout << "Preparing mask... ";
	//halali::prepare_mask(pointcloud, mask, 0.0);
	//mask.export_to_png("Mask.png");


	mask.resize(width, height);
	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			if (pointcloud.at(i, j).z < 1e-3)
			{
				mask.at(i, j) = 255;
			}
			else
			{
				mask.at(i, j) = 0;
			}
		}
	}
	std::cout << "Done." << std::endl;


	//mask.export_to_png("Mask.png");
	std::cout << "Inpainting... ";
	//halali::inpaint(pointcloud, mask);
	halali::fill_holes(pointcloud, mask);
	std::cout << "Done." << std::endl;

	halali::PointCloud pcl_x = pointcloud;
	//halali::PointCloud pcl_y = pointcloud;
	double sigma = 3.0;
	double accuracy = 3.0;


	//halali::gaussian_curvature(pointcloud, 3.0, 3.0);
	//halali::grad_x(pcl_x, sigma, accuracy);
	//halali::grad_y(pcl_y, sigma, accuracy);

	//int hog_x_res =

	//for (int i = )

	//halali::PointCloud pcl_xx = pcl_x;
	//halali::PointCloud pcl_xy = pcl_x;
	//halali::PointCloud pcl_yy = pcl_y;
//
	//std::cout << pcl_xx.size() << std::endl;

	//std::vector<std::vector<double>> sobel_x_kernel;
	//std::vector<std::vector<double>> sobel_y_kernel;
	//halali::get_kernel_sobel_x(sobel_x_kernel);
	//halali::get_kernel_sobel_x(sobel_y_kernel);

	//convolve_value(pcl_xx, sobel_x_kernel);
	//convolve_value(pcl_xy, sobel_y_kernel);
	//convolve_value(pcl_yy, sobel_y_kernel);


	//halali::laplacian(pointcloud, 3.0, 3.0);

	//for (int i = 0; i < pointcloud.size(); i++)
	//{
	//	pointcloud.points[i].value = pcl_x.points[i].value;
	//}

	pointcloud.enable_values(); //Remove before flight, only for exporting height values;

	halali::prepare_mask(pointcloud, mask, 0.0, 8U);

	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			if (mask.at(i, j) == 255)
			{
				pointcloud.at(i, j).z = 0.0;
			}

			pointcloud.at(i, j).value = pointcloud.at(i, j).z; //Remove before flight, only for exporting height values;

			//std::cout << pointcloud.at(i, j).value << std::endl;
		}
	}

	//pointcloud.export_to_ascii("test_inpainted.asc");

	double set_max = 10000;
	double set_min = 9000; //For grad_x/grad_y +-20

	unsigned char* img_out;

	img_out = (unsigned char*)malloc(width * height * sizeof(unsigned char));

	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			unsigned char value;

			if (pointcloud.at(i, j).value < set_min || mask.at(i, j) == 255)
			{
			value = (unsigned char)0;
			}
			else if (pointcloud.at(i, j).value > set_max)
			{
				value = (unsigned char)255;
			}
			else
			{
				value = (unsigned char)round(255 * (pointcloud.at(i, j).value - set_min) / (set_max - set_min));
			}

			img_out[i * width + j] = value;

			//std::cout << value << std::endl;

			//unsigned char value = pointcloud.at(i, j).
		}
	}

	piston::write_png(filename_out_c, img_out, width, height);


	free(img_out);


	//pointcloud.import_from_pcd("pointcloud.pcd");

	//pointcloud.resize(1280, 100);

	//halali::prepare_mask(pointcloud, mask, 100);
	//mask.export_to_png("pointcloud_mask.png");

	//halali::inpaint(pointcloud, mask);

	// mask.export_to_png("pointcloud_mask.png");
	//pointcloud.export_to_ascii("Pointcloud_zMap.pcd");

	return 0;
}


int main3(int argc, char* argv[])
{

	halali::PointCloud pointcloud;
	halali::Image<int> mask;

	//fetch_AT_point_cloud(pointcloud);

	AT::cx::Image range_image;
	std::string filename_in, filename_out;
	char* filename_out_c;
	int label;

	parse_cmd_line_args(argc, argv, filename_in, filename_out, &label);
	filename_out_c = (char*)malloc((filename_out.size() + 1) * sizeof(char));
	strcpy(filename_out_c, filename_out.c_str());

	range_image.load(filename_in);

	std::cout << "Beginning transformation... ";
	rangeimage2pointcloud(range_image, pointcloud);
	std::cout << "Done." << std::endl;

	int width = pointcloud.width();
	int height = pointcloud.height();

	std::cout << "Preparing mask... ";
	//halali::prepare_mask(pointcloud, mask, 0.0);
	//mask.export_to_png("Mask.png");


	mask.resize(width, height);
	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			if (pointcloud.at(i, j).z < 1e-3)
			{
				mask.at(i, j) = 255;
			}
			else
			{
				mask.at(i, j) = 0;
			}
		}
	}
	std::cout << "Done." << std::endl;


	//mask.export_to_png("Mask.png");
	std::cout << "Inpainting... ";
	//halali::inpaint(pointcloud, mask);
	halali::fill_holes(pointcloud, mask);
	std::cout << "Done." << std::endl;

	halali::PointCloud pcl_x = pointcloud;
	//halali::PointCloud pcl_y = pointcloud;
	double sigma = 3.0;
	double accuracy = 3.0;


	//halali::gaussian_curvature(pointcloud, 3.0, 3.0);
	//halali::grad_x(pcl_x, sigma, accuracy);
	//halali::grad_y(pcl_y, sigma, accuracy);

	return 0;
}*/