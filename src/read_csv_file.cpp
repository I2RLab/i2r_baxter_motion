#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
using namespace std;


struct Pos
{
  double t;
  double x_a;
  double y_a;
  double z_a;
  double x_b;
  double y_b;
  double z_b;
  double x_c;
  double y_c;
  double z_c;
  double a_x;
  double a_y;
  double a_z;
  double p;
  double q;
  double r;
};

ostream& operator << (ostream &os, const Pos &rhs)
{
  os << rhs.t   << ','
     << rhs.x_a << ',' << rhs.y_a << ',' << rhs.z_a << ','
     << rhs.x_b << ',' << rhs.y_b << ',' << rhs.z_b << ','
     << rhs.x_c << ',' << rhs.y_c << ',' << rhs.z_c << ','
     << rhs.a_x << ',' << rhs.a_y << ',' << rhs.a_z << ','
     << rhs.p   << ',' << rhs.q << ','   << rhs.r << endl;
  return os;
}

istream& operator >> (istream &is, Pos &rhs)
{
  char delim;
  is >> rhs.t   >> ','
     >> rhs.x_a >> ',' >> rhs.y_a >> ',' >> rhs.z_a >> ','
     >> rhs.x_b >> ',' >> rhs.y_b >> ',' >> rhs.z_b >> ','
     >> rhs.x_c >> ',' >> rhs.y_c >> ',' >> rhs.z_c >> ','
     >> rhs.a_x >> ',' >> rhs.a_y >> ',' >> rhs.a_z >> ','
     >> rhs.p   >> ',' >> rhs.q   >> ',' >> rhs.r;


  is >> rhs.array1
     >> delim
     >> rhs.array2
     >> delim
     >> rhs.red
     >> delim
     >> rhs.blue_o2
     >> delim
     >> rhs.blue_o3;

  return is;
}


int main(int argc, const char *argv[])
{
    if(argc < 2)
    {
      cout << "Usage: " << argv[0] << " filename\n";
//      return 1;
      const char *infilename = 'arm_test1.csv';
    }
    else
    {
      const char *infilename = argv[argc - 1];
    }
    // Reading in the file
    ifstream myfile(infilename);
    if(!myfile)
    {
      cerr << "Couldn't open file " << infilename;
      return 1;
    }

    vector<Pos> whole_list;
    string line;
    while( getline(myfile, line) )
    {
      Pos data;
      stringstream linestr (line);
      linestr >> data;
      whole_list.push_back(data);

      cout << data << '\n';
    }
}