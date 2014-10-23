#include "xytheta_collision_checker.h"

XYThetaCollisionChecker::XYThetaCollisionChecker(const std::vector<sbpl_2Dpt_t> &footprint_polygon, int obs_thresh, int num_heading_disc/*=16*/){
  footprint_polygon_ = footprint_polygon;
  obs_thresh_ = obs_thresh;
  num_heading_disc_ = num_heading_disc;
  grid_ = NULL;
  footprints_initialized_ = false;
  precomputed_footprint_cells.clear();
}

XYThetaCollisionChecker::~XYThetaCollisionChecker(){
  if(grid_ != NULL) delete grid_;
}

void XYThetaCollisionChecker::UpdateOccupancyGrid(nav_msgs::OccupancyGrid new_occ_grid){
  if(grid_ == NULL){
    grid_ = new nav_msgs::OccupancyGrid(new_occ_grid);
    if(!reinit()) ROS_WARN("XYThetaCollisionChecker failed to (re)initialize!");
  } else {
    //we already have a grid!
    if(grid_->info.resolution != new_occ_grid.info.resolution){
      ROS_WARN("Occupancy grid resolution mismatch!");
      delete grid_;
      grid_ = new nav_msgs::OccupancyGrid(new_occ_grid);
      if(!reinit()) ROS_WARN("XYThetaCollisionChecker failed to (re)initialize!");
    } else {
      //update the old grid to the new grid;
      delete grid_;
      grid_ = new nav_msgs::OccupancyGrid(new_occ_grid);
    }
  }
}

double XYThetaCollisionChecker::getCollisionProbability(double x, double y, double theta){
  if(!footprints_initialized_ || grid_ == NULL){
    ROS_WARN_ONCE("Collision checker not initialized!");
    return 0.0;
  }
  return getCollisionProbability(CONTXY2DISC(x, grid_->info.resolution), CONTXY2DISC(y, grid_->info.resolution), ContTheta2Disc(theta, num_heading_disc_));
}

double XYThetaCollisionChecker::getCollisionProbability(int x, int y, int theta){
  if(!footprints_initialized_ || grid_ == NULL){
    ROS_WARN_ONCE("Collision checker not initialized!");
    return 0.0;
  }
  sbpl_2Dcell_t cell;
  EnvNAVXYTHETALAT3Dcell_t interm3Dcell;
  int i;
  double max_cell_prob_ = getCellColllisionProbability(x, y);
  
  //check collisions that for the particular footprint orientation along the action
  if(footprint_polygon_.size() > 1)
  {
    for(i = 0; i < (int)precomputed_footprint_cells[theta].size(); i++) 
    {
      //get the cell in the map
      cell = precomputed_footprint_cells[theta].at(i);
      cell.x = cell.x + x;
      cell.y = cell.y + y;
			
      //check validity
      max_cell_prob_ = std::max(max_cell_prob_, getCellColllisionProbability(cell.x, cell.y));
    }
  }
  return max_cell_prob_; 
}

bool XYThetaCollisionChecker::isValidState(double x, double y, double theta) {
  if(!footprints_initialized_ || grid_ == NULL){
    ROS_WARN_ONCE("Collision checker not initialized!");
    return true;
  }
  return isValidState(CONTXY2DISC(x, grid_->info.resolution), CONTXY2DISC(y, grid_->info.resolution), ContTheta2Disc(theta, num_heading_disc_));
}

bool XYThetaCollisionChecker::IsValidCell(int x, int y){
  if(grid_ == NULL) return false;
  if(x >= grid_->info.width) return false;
  if(y >= grid_->info.height) return false;
  int map_val = grid_->data[y*grid_->info.height + x];
  if(map_val >= obs_thresh_) return false;
  return true;
}

double XYThetaCollisionChecker::getCellColllisionProbability(int x, int y){
  return std::max(0, (int)grid_->data[y*grid_->info.height + x]) / 100.0;
}

bool XYThetaCollisionChecker::isValidState(int x, int y, int theta) {
  if(!footprints_initialized_ || grid_ == NULL){
    ROS_WARN_ONCE("Collision checker not initialized!");
    return true;
  }
  sbpl_2Dcell_t cell;
  EnvNAVXYTHETALAT3Dcell_t interm3Dcell;
  int i;
	
  if(!IsValidCell(x, y))
    return false;

  unsigned char maxcellcost = 0;
  
  //check collisions that for the particular footprint orientation along the action
  if(footprint_polygon_.size() > 1)
  {
    for(i = 0; i < (int)precomputed_footprint_cells[theta].size(); i++) 
    {
      //get the cell in the map
      cell = precomputed_footprint_cells[theta].at(i);
      cell.x = cell.x + x;
      cell.y = cell.y + y;
			
      //check validity
      if(!IsValidCell(cell.x, cell.y))
        return false;

    }
  }
  return true; 
}

bool XYThetaCollisionChecker::reinit(){
  if(grid_ == NULL) return false;

  if(!PreComputeFootprints()){
    return false;
  }

  return true;
}

bool XYThetaCollisionChecker::PreComputeFootprints(){

  precomputed_footprint_cells.clear();

  if(grid_ == NULL){
    ROS_WARN("Occupancy grid not initialized! Can't precompute footprints!");
    footprints_initialized_ = false;
    return false;
  }

  EnvNAVXYTHETALAT3Dpt_t pose;
  pose.x = 0; pose.y = 0; pose.theta = 0;

  precomputed_footprint_cells.resize(num_heading_disc_);

  for(int h = 0; h < num_heading_disc_; h++){
    pose.theta = DiscTheta2Cont(h, num_heading_disc_);
    ROS_INFO("Pre-computing footprint for heading %d/%d", h, num_heading_disc_);
    if(!CalculateFootprintForPose(pose, precomputed_footprint_cells[h], footprint_polygon_)){
      ROS_WARN("Failed to pre-compute footprint polygon for heading %d", h);
      precomputed_footprint_cells.clear();
      footprints_initialized_ = false;
      return false;
    }
  }
  footprints_initialized_ = true;
  return true;
}

bool XYThetaCollisionChecker::CalculateFootprintForPose(EnvNAVXYTHETALAT3Dpt_t pose, std::vector<sbpl_2Dcell_t> &footprint_cells, const std::vector<sbpl_2Dpt_t>& FootprintPolygon)
{
  if(grid_ == NULL) {
    return false;
  }

  int pind;

  //handle special case where footprint is just a point
  if(FootprintPolygon.size() <= 1){
    ROS_WARN("Robot footprint not defined! Using point-robot!");
    sbpl_2Dcell_t cell;
    cell.x = CONTXY2DISC(pose.x, grid_->info.resolution);
    cell.y = CONTXY2DISC(pose.y, grid_->info.resolution);

    for(pind = 0; pind < (int)footprint_cells.size(); pind++)
    {
        if(cell.x == footprint_cells.at(pind).x && cell.y == footprint_cells.at(pind).y)
            break;
    }
    if(pind == (int)footprint_cells.size()) footprint_cells.push_back(cell);
    return true;
  }

  std::vector<sbpl_2Dpt_t> bounding_polygon;
  unsigned int find;
  double max_x = -INFINITECOST, min_x = INFINITECOST, max_y = -INFINITECOST, min_y = INFINITECOST;
  sbpl_2Dpt_t pt = {0,0};
  for(find = 0; find < FootprintPolygon.size(); find++){
    
    //rotate and translate the corner of the robot
    pt = FootprintPolygon[find];
    
    //rotate and translate the point
    sbpl_2Dpt_t corner;
    corner.x = cos(pose.theta)*pt.x - sin(pose.theta)*pt.y + pose.x;
    corner.y = sin(pose.theta)*pt.x + cos(pose.theta)*pt.y + pose.y;
    bounding_polygon.push_back(corner);

    if(corner.x < min_x || find==0){
      min_x = corner.x;
    }
    if(corner.x > max_x || find==0){
      max_x = corner.x;
    }
    if(corner.y < min_y || find==0){
      min_y = corner.y;
    }
    if(corner.y > max_y || find==0){
      max_y = corner.y;
    }
    
  }

  int prev_discrete_x = CONTXY2DISC(pt.x, grid_->info.resolution) + 1; 
  int prev_discrete_y = CONTXY2DISC(pt.y, grid_->info.resolution) + 1;
  prev_discrete_x++; prev_discrete_y++;
  int prev_inside = 0;
  int discrete_x;
  int discrete_y;

  for(double x=min_x; x<=max_x; x+=grid_->info.resolution/3.0){
    for(double y=min_y; y<=max_y; y+=grid_->info.resolution/3.0){
      pt.x = x;
      pt.y = y;
      discrete_x = CONTXY2DISC(pt.x, grid_->info.resolution);
      discrete_y = CONTXY2DISC(pt.y, grid_->info.resolution);
      
      //see if we just tested this point
      if(discrete_x != prev_discrete_x || discrete_y != prev_discrete_y || prev_inside==0){
        if(IsInsideFootprint(pt, &bounding_polygon)){ 
        //convert to a grid point
          sbpl_2Dcell_t cell;
          cell.x = discrete_x;
          cell.y = discrete_y;

          //insert point if not there already
          int pind = 0;
          for(pind = 0; pind < (int)footprint_cells.size(); pind++)
          {
            if(cell.x == footprint_cells.at(pind).x && cell.y == footprint_cells.at(pind).y)
              break;
          }
          if(pind == (int)footprint_cells.size()) footprint_cells.push_back(cell);
          prev_inside = 1;
        } else {
          prev_inside = 0; 
        }
      }     
      prev_discrete_x = discrete_x;
      prev_discrete_y = discrete_y;
    }//over x_min...x_max
  }
  ROS_INFO("Footprint size: %zd cells", footprint_cells.size());
  return true;
}
