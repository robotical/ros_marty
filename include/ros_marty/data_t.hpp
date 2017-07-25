#ifndef DATA_T_HPP
#define DATA_T_HPP

#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <deque>

using namespace std;

struct data_t: deque <deque <float> > {
  typedef deque <deque <float> > ::iterator record_iterator;
  typedef deque        <float>   ::iterator field_iterator;
  bool load( const string& filename );
  bool save( const string& filename );
  bool save( ostream& outs );
};

#endif  /* DATA_T_HPP */
