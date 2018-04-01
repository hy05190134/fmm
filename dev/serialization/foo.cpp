#include <vector>
#include <string>
#include <bitset>
#include <fstream>
#include <boost/serialization/string.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/bitset.hpp>
// ADD THIS!!!
#include <boost/archive/binary_oarchive.hpp>

template<size_t N>
struct Example
{
  std::string id;
  std::vector<std::bitset<N>> bits;
};

template<size_t N>
Example<N> make_example()
{
  Example<N> example;

  example.id = "some id";

  example.bits.resize(100);
  // ADD THIS!!!
  return(example);
}

namespace boost
{
  namespace serialization
  {
    template<typename Archive, size_t N>
    void serialize ( Archive & a
                   , Example<N> & e
                   , const unsigned int version )
    {
        a & e.id;
        a & e.bits;
    }
  }
}

int main()
{
  auto example = make_example<256>();

  std::ofstream ofs("filename", std::ios::binary);

  boost::archive::binary_oarchive oa(ofs);

  oa << example;

  return(0);
}
