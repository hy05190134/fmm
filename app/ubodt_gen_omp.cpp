/**
 * Content
 * Precomputation of UBODT (Optimized)
 *
 * @author: Can Yang
 * @version: 2018.03.09
 */
#include "../src/network_graph_omp.hpp"
#include "../src/config.hpp"
#include <iostream>
#include <ctime>
#include <cstdlib>
using namespace std;
using namespace MM;
int main(int argc, char* argv[])
{
    std::cout<<"------------ Fast map matching (FMM) ------------"<<endl;
    std::cout<<"------------     Author: Can Yang, Modifed: Yang Yang    ------------"<<endl;
    std::cout<<"------------   Version: 2019.09.04   ------------"<<endl;
    std::cout<<"------------  Application: ubodt_gen ------------"<<endl;
    if (argc<4) {
        std::cout<<"argument number error"<<endl;
        std::cout<<"Run `ubodt_gen_omp network_file_path result_file_path delta`"<<endl;
    } else {
        std::string network_file = argv[1];
        std::string result_file = argv[2];
        double delta = atof(argv[3]);
        // clock_t begin_time = clock(); // program start time
        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        std::cout<<"Write UBODT to file "<<result_file<<'\n';
        MM::Network network(network_file,
            "id",
            "source",
            "target");
        MM::NetworkGraphOmp graph(&network);
        std::cout<<"Upperbound config (delta): "<<delta<<'\n';
        bool binary = false;
        graph.precompute_ubodt(result_file, delta, binary);
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        double time_spent = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() / 1000.;
        //double time_spent = (double)(end_time - begin_time) / CLOCKS_PER_SEC;
        std::cout << "Time takes " << time_spent << '\n';
        // clock_t end_time = clock(); // program end time
        // // Unit is second
        // double time_spent = (double)(end_time - begin_time) / CLOCKS_PER_SEC;
        // std::cout<<"Time takes "<<time_spent<<'\n';
    }
    std::cout<<"------------    Program finished     ------------"<<endl;
    return 0;
};
