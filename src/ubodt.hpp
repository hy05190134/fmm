/**
 * Content
 * Definition of UBODT, which is a hashtable containing the precomputed shortest path
 * routing results.
 *
 * @author: Can Yang
 * @version: 2017.11.11
 */

#ifndef MM_UBODT_HPP
#define MM_UBODT_HPP
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <map> /* used for statistics */
#include <fstream>
#include <boost/archive/binary_iarchive.hpp>
#include "types.hpp"
#include "multilevel_debug.h"
// #include <boost/functional/hash.hpp>
namespace MM
{

class UBODT
{
public:
    /**
     *  Constructor of UBODT
     *  @param  buckets: the number of buckets in the hashtable
     *  @param  multipler: multiplier to calculate hashcode of an OD pair as n_o * multiplier + n_d
     */
    UBODT(int buckets_arg,int multiplier_arg):buckets(buckets_arg),multiplier(multiplier_arg) {
        std::cout<<"Creating UBODT with buckets "<< buckets << " muliplier "<< multiplier <<"\n";
        hashtable = (Record **) malloc(sizeof(Record*)*buckets);
        /* This initialization is required to later free the memory, to figure out the problem */
        for (int i = 0; i < buckets; i++){
            hashtable[i] = NULL;
        }
        std::cout<<"Creating UBODT finished\n";
    };

    Record *look_up(int source,int target)
    {
        //int h = (source*multiplier+target)%buckets;
        int h = cal_bucket_index(source,target);
        Record *r = hashtable[h];
        while (r != NULL)
        {
            if (r->source==source && r->target==target)
            {
                return r;
            }
            else
            {
                r=r->next;
            }
        }
        //printf("Not found s %d t %d h %d\n",source,target,h); NULL will be returned.
        return r;
    };
    /**
     *  Return a shortest path (SP) containing edges from source to target.
     *  In case that SP is not found, empty is returned.
     */
    std::vector<int> look_sp_path(int source,int target){
        CPC_DEBUG(4) std::cout<<"Look shortest path from "<< source <<" to "<<target<<'\n';
        std::vector<int> edges;
        if (source==target) {return edges;}
        Record *r=look_up(source,target);
        // No transition exist from source to target
        if (r==NULL){return edges;}
        while(r->first_n!=target){
            edges.push_back(r->next_e);
            r=look_up(r->first_n,target);
        }
        edges.push_back(r->next_e);
        return edges;
    };

    /**
     * Construct the complete path (a vector of edge ID) from an optimal path (a vector of candidates)
     *
     * @param  path, an optimal path
     * @return  a pointer to a complete path, which should be freed by the caller. If there is a large
     * gap in the optimal path implying complete path cannot be found in UBDOT, nullptr is returned
     */
    C_Path *construct_complete_path(O_Path *path){
        CPC_DEBUG(2) std::cout<<"-----------------------"<<'\n';
        CPC_DEBUG(2) std::cout<<"Construct complete path"<<'\n';
        if (path==nullptr) return nullptr;
        C_Path *edges= new C_Path();
        int N = path->size();
        edges->push_back((*path)[0]->edge->id);
        for(int i=0;i<N-1;++i){
            Candidate* a = (*path)[i];
            Candidate* b = (*path)[i+1];
            if ((a->edge->id!=b->edge->id) || (a->offset>b->offset)) {
                auto segs = look_sp_path(a->edge->target,b->edge->source);
                // No transition exist in UBODT
                if (segs.empty() &&  a->edge->target!=b->edge->source){
                CPC_DEBUG(1) std::cout<<"----- Warning: Large gap detected from "<< a->edge->target <<" to "<< b->edge->source <<'\n';
                CPC_DEBUG(1) std::cout<<"----- Construct complete path skipped"<<'\n';
                    delete edges; // free the memory of edges
                    return nullptr;
                }
                for (int e:segs){
                    edges->push_back(e);
                }
                edges->push_back(b->edge->id);
            }
        }
        CPC_DEBUG(2) std::cout<<"Construct complete path finished "<<'\n';
        return edges;
    };

    /**
     * Construct a traversed path from the optimal path.
     * It is different with cpath in that the edges traversed between consecutive GPS observations are Recorded.
     * It returns a traversed path including the cpath and the index of matched edge for each point in the GPS trajectory.
     */
    T_Path *construct_traversed_path(O_Path *path){
        CPC_DEBUG(2) std::cout<<"-----------------------"<<'\n';
        CPC_DEBUG(2) std::cout<<"Construct traversed path begin"<<'\n';
        if (path==nullptr) return nullptr;
        int N = path->size();
        // T_Path *edges= new T_Path();
        T_Path *t_path = new T_Path();
        t_path->cpath.push_back((*path)[0]->edge->id);
        int current_idx = 0;
        t_path->indices.push_back(current_idx);
        for(int i=0;i<N-1;++i){
            Candidate* a = (*path)[i];
            Candidate* b = (*path)[i+1];
            if ((a->edge->id!=b->edge->id) || (a->offset>b->offset)) {
                auto segs = look_sp_path(a->edge->target,b->edge->source);
                // No transition exist in UBODT
                if (segs.empty() &&  a->edge->target!=b->edge->source){
                    CPC_DEBUG(1) std::cout<<"----- Warning: Large gap detected from "<< a->edge->target <<" to "<< b->edge->source <<'\n';
                    CPC_DEBUG(1) std::cout<<"----- Construct complete path skipped"<<'\n';
                    delete t_path; // free the memory of edges
                    return nullptr;
                }
                for (int e:segs){
                    t_path->cpath.push_back(e);
                    ++current_idx;
                }
                t_path->cpath.push_back(b->edge->id);
                ++current_idx;
                t_path->indices.push_back(current_idx);
            } else {
                // b stays on the same edge
                t_path->indices.push_back(current_idx);
            }
        }
        CPC_DEBUG(2) std::cout<<"Construct traversed path finish"<<'\n';
        return t_path;
    };
    /**
     *  Print statistics of the hashtable to a file
     */
    void print_statictics(const char*filename){
        /*
            Iterate through all buckets to get statistics
            Bucket size, counts
        */
        std::map<int,int> statistics;
        for (int i=0;i<buckets;++i){
            int count=0;
            Record *r=hashtable[i];
            while (r!=NULL){
                r=r->next;
                ++count;
            }
            statistics[count]=statistics[count]+1;
        }
        std::ofstream outputfile(filename);
        if (outputfile.is_open())
        {
            outputfile<<"BucketElements;Counts\n";
            for (std::map<int,int>::iterator it=statistics.begin(); it!=statistics.end(); ++it)
                outputfile<< it->first << ";" << it->second << '\n';
            outputfile.close();
        }
        else std::cout << "Unable to write statistics to file"<<'\n';
    };
    double get_delta(){
        return delta;
    };
    inline int cal_bucket_index(int source,int target){
        return (source*multiplier+target)%buckets;
    };
    // inline int cal_bucket_index(int source,int target){
    //     std::size_t seed = source;
    //     boost::hash_combine(seed, target);
    //     return seed%buckets;
    // };
    ~UBODT(){
        /* Clean hashtable */
        std::cout<< "Clean UBODT" << '\n';
        int i;
        for (i=0;i<buckets;++i){
            DEBUG(2) std::cout<<"Clean i "<< i <<'\n';
            Record* head = hashtable[i];
            Record* curr;
            while ((curr = head) != NULL) { // set curr to head, stop if list empty.
                head = head->next;          // advance head to next element.
                free(curr);        // delete saved pointer.
            }
        }
        // Destory hash table pointer
        free(hashtable);
        std::cout<< "Clean UBODT finished" << '\n';
    };
    // Insert a Record into the hash table
    void insert(Record *r)
    {
        //int h = (r->source*multiplier+r->target)%buckets ;
        int h = cal_bucket_index(r->source,r->target);
        r->next = hashtable[h];
        hashtable[h]=r;
        if (r->cost > delta) delta = r->cost;
    };
private:
    const long long multiplier; // multiplier to get a unique ID
    const int buckets; // number of buckets
    double delta = 0.0;
    //int maxnode=0;
    Record** hashtable;
};

double LOAD_FACTOR = 2.0;
int BUFFER_LINE = 1024;

/**
 *  Estimate the number of rows in a UBODT file from its size in byte
 *  @Returns the number of rows
 */
int estimate_ubodt_rows(const std::string &filename){
    struct stat stat_buf;
    int rc = stat(filename.c_str(), &stat_buf);
    if (rc==0) {
        int file_bytes = stat_buf.st_size;
        std::cout<<"UBODT file size is "<<file_bytes<< " bytes\n";
        std::string fn_extension = filename.substr(filename.find_last_of(".") + 1);
        std::transform(fn_extension.begin(), fn_extension.end(), fn_extension.begin(), ::tolower);
        if (fn_extension == "csv" || fn_extension == "txt") {
            int row_size = 36;
            return file_bytes/row_size;
        } else if (fn_extension == "bin" || fn_extension == "binary") {
            Record r;
            // When exporting to a file using boost binary writer,
            // the padding is removed.
            int row_size = 28;
            return file_bytes/row_size;
        }
    } else {
        return -1;
    }
};

int find_prime_number(double value){
    std::vector<int> prime_numbers = {
        5003,10039,20029,50047,100669,200003,500000,
        1000039,2000083,5000101,10000103,20000033};
    int N = prime_numbers.size();
    for (int i=0;i<N;++i){
        if (value<=prime_numbers[i]){
            return prime_numbers[i];
        }
    }
    return prime_numbers[N-1];
};

/* read ubodt from string buffer */
UBODT *read_ubodt_from_str(const std::string &ubodt_str, int multiplier=50000)
{
    std::cout<<"Reading UBODT file from str";

    std::vector<std::string> lines;
    boost::split(lines, ubodt_str, boost::is_any_of("\t"));

    int rows = lines.size();
    std::cout<<"Lys Estimated rows is : " << rows << '\n';
    int progress_step = rows/10;
    if (progress_step < 1) progress_step = 1;
    int buckets = find_prime_number(rows/LOAD_FACTOR);
    UBODT *table = new UBODT(buckets, multiplier);
    int NUM_ROWS = 0;
    char line[BUFFER_LINE];

    // skip header line
    for (int i = 1; i < lines.size(); i++)
    {
        std::string line = lines[i];
        ++NUM_ROWS;
        Record *r =(Record *) malloc(sizeof(Record));
        /* Parse line into a Record */
        sscanf(
            line.c_str(),"%d;%d;%d;%d;%d;%lf",
               &r->source,
               &r->target,
               &r->first_n,
               &r->prev_n,
               &r->next_e,
               &r->cost
        );
        r->next=NULL;
        if (NUM_ROWS%progress_step==0) printf("Read rows: %d\n",NUM_ROWS);
        /* Insert into the hash table */
        table->insert(r);
    };

    std::cout<<"    Number of rows read " << NUM_ROWS << '\n';
    double lf = NUM_ROWS/(double)buckets;
    std::cout<<"    Estimated load factor #elements/#tablebuckets "<<lf<<"\n";
    if (lf>10) std::cout<<"    *** Warning, load factor is too large.\n";
    std::cout<<"Finish reading UBODT.\n";
    return table;
};

/**
 * Read ubodt from a csv file, the caller takes the ownership.
 * The ubodt is stored on heap memory.
 */
UBODT *read_ubodt_csv(const std::string &filename, int multiplier=50000)
{
    std::cout<<"Reading UBODT file (CSV format) from: " << filename << '\n';
    int rows = estimate_ubodt_rows(filename);
    std::cout<<"Estimated rows is : " << rows << '\n';
    int progress_step = rows/10;
    if (progress_step < 1) progress_step = 1;
    int buckets = find_prime_number(rows/LOAD_FACTOR);
    UBODT *table = new UBODT(buckets, multiplier);
    FILE* stream = fopen(filename.c_str(), "r");
    int NUM_ROWS = 0;
    char line[BUFFER_LINE];
    if(fgets(line, BUFFER_LINE, stream)){
        printf("    Header line skipped.\n");
    };
    while (fgets(line, BUFFER_LINE, stream))
    {
        ++NUM_ROWS;
        Record *r =(Record *) malloc(sizeof(Record));
        /* Parse line into a Record */
        sscanf(
            line,"%d;%d;%d;%d;%d;%lf",
               &r->source,
               &r->target,
               &r->first_n,
               &r->prev_n,
               &r->next_e,
               &r->cost
        );
        r->next=NULL;
        if (NUM_ROWS%progress_step==0) printf("Read rows: %d\n",NUM_ROWS);
        /* Insert into the hash table */
        table->insert(r);
    };
    fclose(stream);
    std::cout<<"    Number of rows read " << NUM_ROWS << '\n';
    double lf = NUM_ROWS/(double)buckets;
    std::cout<<"    Estimated load factor #elements/#tablebuckets "<<lf<<"\n";
    if (lf>10) std::cout<<"    *** Warning, load factor is too large.\n";
    std::cout<<"Finish reading UBODT.\n";
    return table;
};

/**
 * Read ubodt from a binary file, the caller takes the ownership.
 */
/*
UBODT *read_ubodt_binary(const std::string &filename, int multiplier=50000)
{
    std::cout<<"Reading UBODT file (binary format) from: " << filename << '\n';
    int rows = estimate_ubodt_rows(filename);
    int progress_step = rows/10;
    if (progress_step < 1) progress_step = 1;
    std::cout<<"Estimated rows is : " << rows << '\n';
    int buckets = find_prime_number(rows/LOAD_FACTOR);
    UBODT *table = new UBODT(buckets,multiplier);
    int NUM_ROWS = 0;
    std::ifstream ifs(filename.c_str());
    // Check byte offset
    std::streampos archiveOffset = ifs.tellg();
    std::streampos streamEnd = ifs.seekg(0, std::ios_base::end).tellg();
    ifs.seekg(archiveOffset);
    boost::archive::binary_iarchive ia(ifs);
    while (ifs.tellg() < streamEnd)
    {
        ++NUM_ROWS;
        Record *r =(Record *) malloc(sizeof(Record));
        ia >> r->source;
        ia >> r->target;
        ia >> r->first_n;
        ia >> r->prev_n;
        ia >> r->next_e;
        ia >> r->cost;
        r->next=NULL;
        if (NUM_ROWS%progress_step==0) printf("Read rows: %d\n",NUM_ROWS);
        // Insert into the hash table
        table->insert(r);
    }
    ifs.close();
    std::cout<<"    Number of rows read " << NUM_ROWS << '\n';
    double lf = NUM_ROWS/(double)buckets;
    std::cout<<"    Estimated load factor #elements/#tablebuckets "<<lf<<"\n";
    if (lf>10) std::cout<<"    *** Warning, load factor is too large.\n";
    std::cout<<"Finish reading UBODT.\n";
    return table;
};
*/
}
#endif /* MM_UBODT_HPP */
