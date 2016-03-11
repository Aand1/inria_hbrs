/*
* @brief : Define struct data types to store information about objects 
*/
namespace semantic_map
{
    struct Position
    { 
	   double x; 
	   double y; 
	   double z; 
    };

    struct Orientation 
    { 
        double x; 
        double y; 
        double z; 
        double w; 
    };

    struct Corner { double x; double y; double z; };

}

