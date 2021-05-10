/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include <leg_tracker/laser_processor.h>


namespace laser_processor
{

Sample* Sample::Extract(int ind, const sensor_msgs::LaserScan& scan)
{
  Sample* s = new Sample();

  s->index = ind;
  s->range = scan.ranges[ind];
  s->x = cos( scan.angle_min + ind*scan.angle_increment ) * s->range;
  s->y = sin( scan.angle_min + ind*scan.angle_increment ) * s->range;
  if (s->range > scan.range_min && s->range < scan.range_max)
  {
    return s;
  }
  else
  {
    delete s;
    return NULL;
  }
}

void SampleSet::clear()
{
  for (SampleSet::iterator i = begin(); i != end(); ++i)
    delete (*i);
  std::set<Sample*, CompareSample>::clear();
}


tf::Point SampleSet::getPosition()
{
  float x_mean = 0.0;
  float y_mean = 0.0;
  for (iterator i = begin(); i != end(); ++i)
  {
    x_mean += ((*i)->x)/size();
    y_mean += ((*i)->y)/size();
  }

  return tf::Point (x_mean, y_mean, 0.0);
}


ScanProcessor::ScanProcessor(const sensor_msgs::LaserScan& scan) 
{
  scan_ = scan;

  SampleSet* cluster = new SampleSet;
  std::vector<int> cluster_indices;
  int number_reject_point=0;
  for (int i = 0; i < scan.ranges.size(); i++)
  {
    Sample* s = Sample::Extract(i, scan);

    if (s != NULL){
      cluster->insert(s);
      cluster_indices.push_back(i);
    }
    else{
      number_reject_point++;
    }
  }
  clusters_.push_back(cluster);
  cluster_indices_.push_back(cluster_indices);
  printf("%ld ---- %ld --- number_reject_point: %d --- total_point: %ld\n",cluster_indices.size(),cluster->size(),number_reject_point,scan.ranges.size());
}

ScanProcessor::~ScanProcessor()
{
  for ( std::list<SampleSet*>::iterator c = clusters_.begin(); c != clusters_.end(); ++c)
    delete (*c);
}

void ScanProcessor::removeLessThan(uint32_t num)
{
  printf("removeLessThan\n");
  std::list<SampleSet*>::iterator c_iter = clusters_.begin();

  std::vector<int> indexToBeRemove;
  printf("Before removeLessThan function: %ld -- %ld -- %d -- %ld\n",clusters_.size(),laser_indices.size()-1,laser_indices[laser_indices.size()-1],cluster_indices_.begin()->size()-1);
  for(int i=0;i<laser_indices.size()-1;i++){
    printf("From index: %d to %d\n",laser_indices[i],laser_indices[i+1]);
  }

  int index = 0;
  int removed_points = 0;
  while (c_iter != clusters_.end())
  {
    if ( (*c_iter)->size() < num )
    {
      printf("Deleting numberOfPoint: %ld from %d --> %d\n",(*c_iter)->size(),laser_indices[index],laser_indices[index+1]);
      indexToBeRemove.push_back(laser_indices[index]);
      indexToBeRemove.push_back(laser_indices[index+1]);
      removed_points+=(*c_iter)->size();
      delete (*c_iter);
      clusters_.erase(c_iter++);
    } 
    else 
    {
      printf("Current segment: %ld\n",(*c_iter)->size());
      ++c_iter;
    }
    index++;
  }
  // Don't forget the last index
  if(abs(laser_indices[index]-laser_indices[index-1])<num){
    indexToBeRemove.push_back(laser_indices[index-1]);
    indexToBeRemove.push_back(laser_indices[index]);
  }

  printf("laser_indices size %ld -- %d -- %d\n",laser_indices.size(),index-1,index);  
  printf("indexToBeRemove size %ld\n",indexToBeRemove.size());  

  printf("laser_indices: ");
  for(int i = 0;i<laser_indices.size();i++){
    printf("%d--",laser_indices[i]);
  }
  printf("\n");

  printf("indexToBeRemove: ");
  for(int i = 0;i<indexToBeRemove.size();i++){
    printf("%d--",indexToBeRemove[i]);
  }
  printf("\n");

  std::list<std::vector<int>>::iterator cluster_indices__indices = cluster_indices_.begin();
  while (cluster_indices__indices != cluster_indices_.end())
  {
    if(indexToBeRemove.size()>1){
      for(int i=0;i<indexToBeRemove.size()-1;){
        printf("indexToBeRemove: %d to %d\n",indexToBeRemove[i],indexToBeRemove[i+1]);
        i=i+2;
      }

      int removed_indices = 0;
      for(int i=0;i<indexToBeRemove.size()-1;){

        for(int no_cluster_indices__indices=0;no_cluster_indices__indices<100;no_cluster_indices__indices++){
          printf("%d--",(*cluster_indices__indices)[no_cluster_indices__indices]);
        }

        printf("\nlaser_indices: ");
        for(int i = 0;i<laser_indices.size();i++){
          printf("%d--",laser_indices[i]);
        }

        printf("\n");
        printf("cluster_indices_ size: %ld\n",cluster_indices_.begin()->size());
        printf("Before From index: %d to %d\n",indexToBeRemove[i],indexToBeRemove[i+1]);
        printf("After From index: %d to %d\n",indexToBeRemove[i]-removed_indices,indexToBeRemove[i+1]-removed_indices);

        (*cluster_indices__indices).erase((*cluster_indices__indices).begin()+indexToBeRemove[i]-removed_indices,(*cluster_indices__indices).begin()+indexToBeRemove[i+1]-removed_indices);
        auto it = find(laser_indices.begin(), laser_indices.end(), indexToBeRemove[i+1]);
        // if (it != laser_indices.end())
        // {
        //   laser_indices.erase(std::remove(laser_indices.begin(), laser_indices.end(), indexToBeRemove[i+1]), laser_indices.end());
          
        //   int index = it - laser_indices.begin();
        //   printf("Found index %d with value: %d with removed_indices: %d\n",index,indexToBeRemove[i+1],removed_indices);

        //   for(int i = index;i<laser_indices.size();i++){
        //     laser_indices[i] = laser_indices[i] - indexToBeRemove[i+1];
        //   }
        // }
        // else printf("Not found!!!!!!!!! %d %d\n",indexToBeRemove[i],indexToBeRemove[i+1]);
        
        removed_indices += indexToBeRemove[i+1] - indexToBeRemove[i];

        i=i+2;
      }
    }


      printf("------------------------------------------------- Finish processing!!!!!\n");
      // for(int no_cluster_indices__indices=0;no_cluster_indices__indices<(*cluster_indices__indices).size()-1;no_cluster_indices__indices++){
      //   printf("%d--",(*cluster_indices__indices)[no_cluster_indices__indices]);
      // }

      // std::vector<int> laser_indices;
      laser_indice_cluster.clear();
      laser_indices.clear();

      int sum_indices=0;
      laser_indices.push_back(0);

      c_iter = clusters_.begin();
      while (c_iter != clusters_.end())
      {
        laser_indices.push_back((*c_iter)->size()+sum_indices);
        sum_indices+=(*c_iter)->size();
        ++c_iter;
      }
      printf("\nlaser_indices: ");
      for(int i = 0;i<laser_indices.size()-1;i++){
        printf("%d--",laser_indices[i]);
      }
      printf("\n");
      for(int i = 0;i<laser_indices.size()-1;i++){
        printf("This cluster from %d to %d\n",laser_indices[i],laser_indices[i+1]-1);
        std::vector<uint32_t> current_cluster_indices;
        for(int no_cluster_indices__indices=laser_indices[i];no_cluster_indices__indices<laser_indices[i+1];no_cluster_indices__indices++){
          current_cluster_indices.push_back((*cluster_indices__indices)[no_cluster_indices__indices]);
          printf("%d--",(*cluster_indices__indices)[no_cluster_indices__indices]);
        }
        laser_indice_cluster.push_back(current_cluster_indices);
        printf("\npush back to laser_indice_cluster: %ld\n",current_cluster_indices.size());
      }
      ++cluster_indices__indices;
  }

  printf("After removed_cluster: %d  -- laser_indice_cluster: %ld\n",removed_points,laser_indice_cluster.size());
  printf("After removeLessThan %ld -- %ld -- %d -- %ld\n",clusters_.size(),laser_indices.size()-1,laser_indices[laser_indices.size()-1],cluster_indices_.begin()->size()-1);
  printf("-----\n");
}


void ScanProcessor::splitConnected(float thresh)
{
  // Holds our temporary list of split clusters 
  // because we will be modifying our existing list in the mean time
  std::list<SampleSet*> tmp_clusters;
  laser_indices.clear();

  std::list<SampleSet*>::iterator c_iter = clusters_.begin();

  while (c_iter != clusters_.end())
  {
    int initiate_point=(*c_iter)->size();
    while ((*c_iter)->size() > 0)
    {
      // Iterate over laser scan samples in clusters_
      // and collect those which are within a euclidian distance of <thresh>
      // and store new clusters in tmp_clusters
      SampleSet::iterator s_first = (*c_iter)->begin();
      std::list<Sample*> sample_queue;
      sample_queue.push_back(*s_first);
      (*c_iter)->erase(s_first);
      std::list<Sample*>::iterator s_q = sample_queue.begin();
      // printf("------- start iterating point index: %ld-------\n",initiate_point-(*c_iter)->size()-1);
      laser_indices.push_back(initiate_point-(*c_iter)->size()-1);
      while (s_q != sample_queue.end())
      {
        int expand = (int)(asin( thresh / (*s_q)->range ) / scan_.angle_increment);

        SampleSet::iterator s_rest = (*c_iter)->begin();

        while ( (s_rest != (*c_iter)->end() and (*s_rest)->index < (*s_q)->index + expand ) )
        {
          if (sqrt( pow( (*s_q)->x - (*s_rest)->x, 2.0f) + pow( (*s_q)->y - (*s_rest)->y, 2.0f)) < thresh)
          {
            sample_queue.push_back(*s_rest);
            (*c_iter)->erase(s_rest++);
          } 
          else 
          {
            ++s_rest;
          }  
        }
        s_q++;
      }

      // Move all the samples into the new cluster
      SampleSet* c = new SampleSet;
      for (s_q = sample_queue.begin(); s_q != sample_queue.end(); s_q++)
        c->insert(*s_q);

      // Store the temporary clusters
      tmp_clusters.push_back(c);
    }

    // printf("------- end iterating point index: %d-------\n",initiate_point-1);
    laser_indices.push_back(initiate_point-1);
    //Now that c_iter is empty, we can delete
    delete (*c_iter);

    //And remove from the map
    clusters_.erase(c_iter++);
  }

  // printf("clusters_ size: %ld  ||  tmp_clusters: %ld\n",clusters_.size(),tmp_clusters.size());
  // Insert our temporary clusters list back into the de facto list
  clusters_.insert(clusters_.begin(), tmp_clusters.begin(), tmp_clusters.end());
}
}; // namespace laser_processor 
