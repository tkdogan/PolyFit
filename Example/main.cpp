/*
Copyright (C) 2017  Liangliang Nan
https://3d.bk.tudelft.nl/liangliang/ - liangliang.nan@gmail.com

Timur Dogan (tkdogan@cornell.edu): added commandline options for use as console application

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/


#include "../basic/logger.h"
#include "../model/point_set.h"
#include "../model/map.h"
#include "../method/method_global.h"
#include "../method/hypothesis_generator.h"
#include "../method/face_selection.h"
#include "../model/map_io.h"
#include "../model/point_set_io.h"

// added for commandline options
#include <boost/program_options.hpp>
#include <iostream>
using namespace boost::program_options;


int main(int argc, char** argv)
{
	std::cout << "PolyFit Commandline Tool" << '\n';

	// polyfit parameters
	float lambda_data_fitting = 0.43;
	float lambda_model_coverage = 0.27;
	float lambda_model_complexity = 0.3;
	// input point cloud file name
	std::string input_file = std::string("in.vg");
	// output mesh file name
	std::string output_file = std::string("out.obj");;


	try
	{
		options_description desc("Options for PolyFit");
		desc.add_options()
			("help,h", "Help screen")

			("input,i", value<std::string>(&input_file)->default_value("in.vg"), "input file path (*.vg)")
			("output,o", value<std::string>(&output_file)->default_value("out.obj"), "output file path (*.obj)")

			("lambda_data_fitting,f",  value<float>(&lambda_data_fitting)->default_value(0.43), "lambda_data_fitting")
			("lambda_model_coverage,c",  value<float>(&lambda_model_coverage)->default_value(0.27), "lambda_model_coverage")
			("lambda_model_complexity,x",  value<float>(&lambda_model_complexity)->default_value(0.3), "lambda_model_complexity")
			;

		variables_map vm;
		store(parse_command_line(argc, argv, desc), vm);
		notify(vm);

		if (vm.count("help"))
			std::cout << desc << '\n';

		else if (vm.count("input"))
			input_file = vm["input"].as<std::string>();
		else if (vm.count("output"))
			output_file = vm["output"].as<std::string>();

		else if (vm.count("lambda_data_fitting"))
			lambda_data_fitting = vm["lambda_data_fitting"].as<float>();
		else if (vm.count("lambda_model_coverage"))
			lambda_model_coverage = vm["lambda_model_coverage"].as<float>();
		else if (vm.count("lambda_model_complexity"))
			lambda_model_complexity = vm["lambda_model_complexity"].as<float>();
	}
	catch (const error& ex)
	{
		std::cerr << ex.what() << '\n';
	}



	//std::cout << "Arguments: inputFile.vg outputFile.obj" << std::endl;

	//std::cout << "Have " << argc << " arguments:" << std::endl;
	//for (int i = 0; i < argc; ++i) {
	//	std::cout << argv[i] << std::endl;
	//}

	//// input point cloud file name
	//std::string input_file = std::string(argv[1]);
	//// output mesh file name
	//std::string output_file = std::string(argv[2]);;


	// initialize the logger (this is not optional)
	Logger::initialize();

	//// input point cloud file name
	//const std::string input_file = std::string(POLYFIT_ROOT_DIR) + "/toy_data.bvg";
	//// output mesh file name
	//const std::string output_file = std::string(POLYFIT_ROOT_DIR) + "/toy_data-result.obj";

	//// below are the default parameters (change these when necessary)
	//Method::lambda_data_fitting = 0.43;
	//Method::lambda_model_coverage = 0.27;
	//Method::lambda_model_complexity = 0.3;
	Method::lambda_data_fitting = lambda_data_fitting;
	Method::lambda_model_coverage = lambda_model_coverage;
	Method::lambda_model_complexity = lambda_model_complexity;

	// load point cloud from file
	PointSet* pset = PointSetIO::read(input_file);
	if (!pset) {
		std::cerr << "failed loading point cloud from file: " << input_file << std::endl;
		return EXIT_FAILURE;
	}

	// step 1: refine planes
	std::cout << "refining planes..." << std::endl;
	const std::vector<VertexGroup::Ptr>& groups = pset->groups();
	if (groups.empty()) {
		std::cerr << "planar segments do not exist" << std::endl;
		return EXIT_FAILURE;
	}
	HypothesisGenerator hypothesis(pset);
	hypothesis.refine_planes();

	// step 2: generate face hypothesis
	std::cout << "generating plane hypothesis..." << std::endl;
	Map* mesh = hypothesis.generate();
	if (!mesh) {
		std::cerr << "failed generating candidate faces. Please check if the input point cloud has good planar segments" << std::endl;
		return EXIT_FAILURE;
	}
	hypothesis.compute_confidences(mesh, false);

	// step 3: face selection
	std::cout << "optimization..." << std::endl;
	const auto& adjacency = hypothesis.extract_adjacency(mesh);
	FaceSelection selector(pset, mesh);
	selector.optimize(adjacency, LinearProgramSolver::SCIP);
	if (mesh->size_of_facets() == 0) {
		std::cerr << "optimization failed: model has on faces" << std::endl;
		return EXIT_FAILURE;
	}
	// now we don't need the point cloud anymore, and it can be deleted
	delete pset;

	// step 4: save result to file
	if (MapIO::save(output_file, mesh))
		std::cout << "reconstructed model saved to file: " << output_file << std::endl;
	else {
		std::cerr << "failed saving reconstructed model to file: " << output_file << std::endl;
		return EXIT_FAILURE;
	}

	return EXIT_SUCCESS;
};


