/**
 * Content
 * FMM application (by stream)
 *
 * @author: Can Yang
 * @version: 2017.11.11
 */
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <fstream>
#include <string>
#include <ctime>
#include "../src/network.hpp"
#include "../src/ubodt.hpp"
#include "../src/transition_graph.hpp"
#include "../src/gps.hpp"
#include "../src/reader.hpp"
#include "../src/writer.hpp"
#include "../src/multilevel_debug.h"
#include "../src/config.hpp"
using namespace std;
using namespace MM;
using namespace MM::IO;

int main (int argc, char **argv)
{
    if (argc<8)
    {
        std::cout<<"augument number error"<<endl;
        std::cout<<"Run `fmm network_file_path ubodt_file_path gps_string k radius gps_error penalty_factor`"<<endl;
    } else {
        std::string network_file = argv[1];
        std::string ubodt_file = argv[2];
        std::string wkt = argv[3];
        double k = atof(argv[4]);
        double radius = atof(argv[5]);
        double gps_error = atof(argv[6]);
        double penalty_factor = atof(argv[7]);

        // clock_t begin_time = clock(); // program start time
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        Network network(network_file, "id", "source", "target");
        network.build_rtree_index();
        int multiplier = network.get_node_count();
        if (multiplier==0) multiplier = 50000;

        UBODT *ubodt = read_ubodt_csv(ubodt_file,multiplier);
        double delta = ubodt->get_delta();

        // get all result
        ResultConfig result_config;
        result_config.write_mgeom = true;

        // The header is moved to constructor of result writer
        // rw.write_header();

        LineString line;
        stringstream ss;
        bg::read_wkt(wkt,*(line.get_geometry()));

        int points_in_tr = line.getNumPoints();
        // Candidate search
        Traj_Candidates traj_candidates = network.search_tr_cs_knn(&line,k,radius,gps_error);
        TransitionGraph tg = TransitionGraph(&traj_candidates,&line,ubodt,delta);
        // Optimal path inference
        O_Path *o_path_ptr = tg.viterbi(penalty_factor);
        // Complete path construction as an array of indices of edges vector
        T_Path *t_path_ptr = ubodt->construct_traversed_path(o_path_ptr);
        // C_Path *c_path_ptr = ubodt->construct_complete_path(o_path_ptr);
        if (result_config.write_mgeom) {
            LineString *m_geom = network.complete_path_to_geometry(o_path_ptr,&(t_path_ptr->cpath));
            ResultWriter::write_geometry(ss, m_geom);
            cerr << ss.str() << endl;
            delete m_geom;
        }

        delete o_path_ptr;
        delete t_path_ptr;
        delete ubodt;
    }

    return 0;
};
