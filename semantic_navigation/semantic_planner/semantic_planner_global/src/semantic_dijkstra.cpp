#include <semantic_planner_global/semantic_dijkstra.h>
#include <algorithm>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

typedef boost::geometry::model::d2::point_xy<double> point_type;
typedef boost::geometry::model::polygon<point_type> polygon_type;

using namespace semantic_planner; 

SemanticDijkstra::SemanticDijkstra(costmap_2d::Costmap2D* costmap, global_planner::PotentialCalculator* p_calc, int nx, int ny) :
        global_planner::Expander(p_calc, nx, ny), costmap_(costmap), pending_(NULL), precise_(false), lethal_cost_(253.0), neutral_cost_(50.0) 
{
    // priority buffers
    buffer1_ = new int[PRIORITYBUFSIZE];
    buffer2_ = new int[PRIORITYBUFSIZE];
    buffer3_ = new int[PRIORITYBUFSIZE];

    priorityIncrement_ = 2 * neutral_cost_;
}

SemanticDijkstra::~SemanticDijkstra() 
{
  delete[] buffer1_;
  delete[] buffer2_;
  delete[] buffer3_;
  if (pending_)
      delete[] pending_;
}

//
// Set/Reset map size
//
void SemanticDijkstra::setSize(int xs, int ys) {
    global_planner::Expander::setSize(xs, ys);
    if (pending_)
        delete[] pending_;

    pending_ = new bool[ns_];
    memset(pending_, 0, ns_ * sizeof(bool));
}

//
// main propagation function
// Dijkstra method, breadth-first
// runs for a specified number of cycles,
//   or until it runs out of cells to update,
//   or until the Start cell is found (atStart = true)
//

bool SemanticDijkstra::computePotentials(unsigned char* costs, double start_x, double start_y, double end_x, double end_y,
                                           int cycles, float* potential, semantic_map::Object& object) {
    object_  = object;
    cells_visited_ = 0;
    // priority buffers
    threshold_ = lethal_cost_;
    currentBuffer_ = buffer1_;
    currentEnd_ = 0;
    nextBuffer_ = buffer2_;
    nextEnd_ = 0;
    overBuffer_ = buffer3_;
    overEnd_ = 0;
    memset(pending_, 0, ns_ * sizeof(bool));
    std::fill(potential, potential + ns_, POT_HIGH);

    // set goal
    int k = toIndex(start_x, start_y);
    //precise_ = false; 
    if(precise_)
    {
        double dx = start_x - (int)start_x, dy = start_y - (int)start_y;
        dx = floorf(dx * 100 + 0.5) / 100;
        dy = floorf(dy * 100 + 0.5) / 100;
        potential[k] = neutral_cost_ * 2 * dx * dy;
        potential[k+1] = neutral_cost_ * 2 * (1-dx)*dy;
        potential[k+nx_] = neutral_cost_*2*dx*(1-dy);
        potential[k+nx_+1] = neutral_cost_*2*(1-dx)*(1-dy);//*/

        push_cur(k+2);
        push_cur(k-1);
        push_cur(k+nx_-1);
        push_cur(k+nx_+2);

        push_cur(k-nx_);
        push_cur(k-nx_+1);
        push_cur(k+nx_*2);
        push_cur(k+nx_*2+1);
    }else{
        potential[k] = 0;
        push_cur(k+1);
        push_cur(k-1);
        push_cur(k-nx_);
        push_cur(k+nx_);
    }

    int nwv = 0;            // max priority block size
    int nc = 0;            // number of cells put into priority blocks
    int cycle = 0;        // which cycle we're on

    // set up start cell
    int startCell = toIndex(end_x, end_y);

    for (; cycle < cycles; cycle++) // go for this many cycles, unless interrupted
            {
        // 
        if (currentEnd_ == 0 && nextEnd_ == 0) // priority blocks empty
            return false;

        // stats
        nc += currentEnd_;
        if (currentEnd_ > nwv)
            nwv = currentEnd_;

        // reset pending_ flags on current priority buffer
        int *pb = currentBuffer_;
        int i = currentEnd_;
        while (i-- > 0)
            pending_[*(pb++)] = false;

        // process current priority buffer
        pb = currentBuffer_;
        i = currentEnd_;
        while (i-- > 0)
            updateCell(costs, potential, *pb++);

        // swap priority blocks currentBuffer_ <=> nextBuffer_
        currentEnd_ = nextEnd_;
        nextEnd_ = 0;
        pb = currentBuffer_;        // swap buffers
        currentBuffer_ = nextBuffer_;
        nextBuffer_ = pb;

        // see if we're done with this priority level
        if (currentEnd_ == 0) {
            threshold_ += priorityIncrement_;    // increment priority threshold
            currentEnd_ = overEnd_;    // set current to overflow block
            overEnd_ = 0;
            pb = currentBuffer_;        // swap buffers
            currentBuffer_ = overBuffer_;
            overBuffer_ = pb;
        }

        // check if we've hit the Start cell
        if (potential[startCell] < POT_HIGH)
            break;
    }
    //ROS_INFO("CYCLES %d/%d ", cycle, cycles);
    if (cycle < cycles)
        return true; // finished up here
    else
        return false;
}

//
// Critical function: calculate updated potential value of a cell,
//   given its neighbors' values
// Planar-wave update calculation from two lowest neighbors in a 4-grid
// Quadratic approximation to the interpolated value
// No checking of bounds here, this function should be fast
//

#define INVSQRT2 0.707106781

inline void SemanticDijkstra::updateCell(unsigned char* costs, float* potential, int n) 
{
    cells_visited_++;

    // do planar wave update
    float c = getCost(costs, n);

    if (c >= lethal_cost_)    // don't propagate into obstacles
        return;
    

    float pot = p_calc_->calculatePotential(potential, c, n);
    //ROS_INFO_STREAM(pot);

    // now add affected neighbors to priority blocks
    if (pot < potential[n]) 
    {
        float le = INVSQRT2 * (float)getCost(costs, n - 1);
        float re = INVSQRT2 * (float)getCost(costs, n + 1);
        float ue = INVSQRT2 * (float)getCost(costs, n - nx_);
        float de = INVSQRT2 * (float)getCost(costs, n + nx_);
        potential[n] = pot;
        //ROS_INFO("UPDATE %d %d %d %f", n, n%nx, n/nx, potential[n]);
        if (pot < threshold_)    // low-cost buffer block
        {
            if (potential[n - 1] > pot + le)
                push_next(n-1);
            if (potential[n + 1] > pot + re)
                push_next(n+1);
            if (potential[n - nx_] > pot + ue)
                push_next(n-nx_);
            if (potential[n + nx_] > pot + de)
                push_next(n+nx_);
        } 
        else            // overflow block
        {
            if (potential[n - 1] > pot + le)
                push_over(n-1);
            if (potential[n + 1] > pot + re)
                push_over(n+1);
            if (potential[n - nx_] > pot + ue)
                push_over(n-nx_);
            if (potential[n + nx_] > pot + de)
                push_over(n+nx_);
        }
    }
}

bool SemanticDijkstra::isMovableObject(int n, semantic_map::Object& object)
{ 
    polygon_type poly;

    unsigned int mx_, my_; // associated map coordinates of the costmap index under consideration.
    unsigned int omx_, omy_; // associated map coordinates of the object bounding box descibed in world coordinates.
    double owx_, owy_; // world coordinates of the object to be converted into world coordinates.
    
    costmap_->indexToCells(n, mx_, my_); 

    for (l_ = 0; l_ < 4; l_++) 
    {
        owx_ = object.geometry.bounding_box.vertices[l_].x;
        owy_ =  object.geometry.bounding_box.vertices[l_].y;

        costmap_->worldToMap(owx_, owy_, omx_, omy_);

        polyX[l_] = omx_;
        polyY[l_] = omy_;
        ROS_INFO_STREAM(omx_);
        ROS_INFO_STREAM(omy_);

        poly.outer().push_back(point_type(omx_,omy_));

        
    }
    
    inside_ = false;
    near_ = false;  
    oddNodes_= false;
    j_ = 3;
      
    for (i_=0; i_<4; i_++) 
    {
        if ( (polyY[i_] < my_ && polyY[j_] >= my_ || polyY[j_] < my_ && polyY[i_] >= my_)
           &&  (polyX[i_] <= mx_ || polyX[j_] <= mx_) ) 
        {
            if (polyX[i_]+(my_-polyY[i_])/(polyY[j_]-polyY[i_])*(polyX[j_]-polyX[i_]) < mx_) 
            {
                oddNodes_ = !oddNodes_; 
            }
        }
        
        j_ = i_; 
    }
   
    if (oddNodes_ == true)
    {
        inside_ = true;
    }

    ////////////////////////////////////////////////////////
    point_type p(mx_,my_);

    double distance = boost::geometry::distance(p, poly);

    if (distance <= 100.0)
        near_ = true;
        //ROS_INFO_STREAM("INSIDE");

    bool result = (inside_ || near_);
    //ROS_INFO_STREAM(result);

    return result;
  
}

