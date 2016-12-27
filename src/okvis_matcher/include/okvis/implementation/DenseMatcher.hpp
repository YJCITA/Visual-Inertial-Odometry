/*********************************************************************************
 *  OKVIS - Open Keyframe-based Visual-Inertial SLAM
 *  Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 * 
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *   * Neither the name of Autonomous Systems Lab / ETH Zurich nor the names of
 *     its contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Created on: 2013
 *      Author: Simon Lynen
 *    Modified: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *********************************************************************************/

/**
 * @file implementation/DenseMatcher.hpp
 * @brief Header implementation file for the DenseMatcher class.
 * @author Simon Lynen
 * @author Stefan Leutenegger
 */

#include <map>

/// \brief okvis Main namespace of this package.
namespace okvis {

// This function creates all the matching threads and assigns the best matches afterwards.
template<typename MATCHING_ALGORITHM_T>
// doWorkPtr 实际调用的是 doWorkLinearMatching
void DenseMatcher::matchBody(
    void (DenseMatcher::*doWorkPtr)(MatchJob&, MATCHING_ALGORITHM_T*),
    MATCHING_ALGORITHM_T& matchingAlgorithm) {
  // create lock list
  // locks 和 frameB 中特征个数一致
  std::mutex* locks = new std::mutex[matchingAlgorithm.sizeB()];

  //the pairing list
  // vpairs 的维度和 frameB 中的特征个数一致，对于 frameB 中的特征，在 frameA 中找匹配
  pairing_list_t vpairs;
  // a list with best matches for each "A" point
  // frameA 中的点在 frameB 中找几个匹配点，所以是两层 vector
  std::vector<std::vector<pairing_t> > vMyBest;

  vMyBest.resize(matchingAlgorithm.sizeA());

  // this point is not paired so far, score max
  // vpairs 和 frameB 中的特征个数一致
  vpairs.resize(matchingAlgorithm.sizeB(),
                pairing_t(-1, std::numeric_limits<distance_t>::max()));

  // prepare the jobs for the threads
  std::vector<MatchJob> jobs(numMatcherThreads_);
  for (int i = 0; i < numMatcherThreads_; ++i) {
    jobs[i].iThreadID = i;
    jobs[i].vpairs = &vpairs;
    jobs[i].vMyBest = &vMyBest;
    jobs[i].mutexes = locks;
  }

  //create all threads
  //  boost::thread_group matchers;
  for (int i = 0; i < numMatcherThreads_; ++i) {
    matcherThreadPool_->enqueue(doWorkPtr, this, jobs[i], &matchingAlgorithm);
    //    matchers.create_thread(boost::bind(doWorkPtr, this, jobs[i], &matchingAlgorithm));
  }

  //  matchers.join_all();
  matcherThreadPool_->waitForEmptyQueue();

  // Looks like running this in one thread is faster than creating 30+ new threads for every image.
  //TODO(gohlp): distribute this to n threads.

  //  for (int i = 0; i < _numMatcherThreads; ++i)
  //  {
  //	  (this->*doWorkPtr)(jobs[i], &matchingAlgorithm);
  //  }

  matchingAlgorithm.reserveMatches(vpairs.size());

  // assemble the pairs and return
  // const_distratiothres 比例阈值
  const distance_t& const_distratiothres = matchingAlgorithm.distanceRatioThreshold();
  const distance_t& const_distthres = matchingAlgorithm.distanceThreshold();
  for (size_t i = 0; i < vpairs.size(); ++i) {
    // vpairs[i] 存储了两项，一项是 frameB 中的特征 i 在 frameA 的匹配，另一项是匹配描述子的距离
    // 这里 useDistanceRatioThreshold_ = false，程序里面没用
    if (useDistanceRatioThreshold_ && vpairs[i].distance < const_distthres) {
      // frameA 中的特征 vpairs[i] 在 frameB 中描述子最小的特征
      const std::vector<pairing_t>& best_matches_list =
          vMyBest[vpairs[i].indexA];
      OKVIS_ASSERT_TRUE_DBG(Exception, best_matches_list[0].indexA != -1,
                            "assertion failed");

      if (best_matches_list[1].indexA != -1) {
        const distance_t& best_match_distance = best_matches_list[0].distance;
        const distance_t& second_best_match_distance = best_matches_list[1]
            .distance;
        // Only assign if the distance ratio better than the threshold.
        // 如果描述子距离第二小的和描述子距离最小的比值大于一定阈值，那么设置 vpairs[i] 和 i 匹配
        // 这里似乎有些不合逻辑， 应该设置 vpairs[i] 和 best_matches_list[0].indexA 匹配
        if (best_match_distance == 0
            || second_best_match_distance / best_match_distance
                > const_distratiothres) {
          matchingAlgorithm.setBestMatch(vpairs[i].indexA, i,
                                         vpairs[i].distance);
        }
      } else {
        // If there is only one matching feature, we assign it.
        // 如果只有一个最佳匹配
        matchingAlgorithm.setBestMatch(vpairs[i].indexA, i, vpairs[i].distance);
      }
    }  // frameB 中特征 i 和 frameA 中特征 vpairs[i] 描述子小于一定阈值，则设置最佳匹配
    else if (vpairs[i].distance < const_distthres) {
      matchingAlgorithm.setBestMatch(vpairs[i].indexA, i, vpairs[i].distance);
    }
  }

  delete[] locks;
}

/*
MATCHING_ALGORITHM_T 和 matchingAlgorithm 类型：
VioKeyframeWindowMatchingAlgorithm<
                okvis::cameras::PinholeCamera<
                    okvis::cameras::RadialTangentialDistortion> > >
*/
// Execute a matching algorithm. This is the fast, templated version. Use this.
template<typename MATCHING_ALGORITHM_T>
void DenseMatcher::match(MATCHING_ALGORITHM_T & matchingAlgorithm) {
  typedef MATCHING_ALGORITHM_T matching_algorithm_t;
  // 调用的是 VioKeyframeWindowMatchingAlgorithm 中的 
  // VioKeyframeWindowMatchingAlgorithm<CAMERA_GEOMETRY_T>::doSetup
  matchingAlgorithm.doSetup();

  // call the matching body with the linear matching function pointer
  matchBody(&DenseMatcher::template doWorkLinearMatching<matching_algorithm_t>,
            matchingAlgorithm);
}

// Execute a matching algorithm implementing image space matching.
template<typename MATCHING_ALGORITHM_T>
void DenseMatcher::matchInImageSpace(MATCHING_ALGORITHM_T & matchingAlgorithm) {
  typedef MATCHING_ALGORITHM_T matching_algorithm_t;
  matchingAlgorithm.doSetup();

  // call the matching body with the image space matching function pointer
  matchBody(
      &DenseMatcher::template doWorkImageSpaceMatching<matching_algorithm_t>,
      matchingAlgorithm);
}

// This calculates the distance between to keypoint descriptors. If it is better than the /e numBest_
// found so far, it is included in the aiBest list.
// aiBest 存储 frameA 中的特征 shortindexA 在 frameB 中特征描述子最小的 numBest_ 个特征的编号
template<typename MATCHING_ALGORITHM_T>
inline void DenseMatcher::listBIteration(
    MATCHING_ALGORITHM_T* matchingAlgorithm, std::vector<pairing_t>& aiBest,
    size_t shortindexA, size_t i) {
  OKVIS_ASSERT_TRUE(std::runtime_error, matchingAlgorithm != NULL,
                    "matching algorithm is NULL");
  typename DenseMatcher::distance_t tmpdist;

  // is this better than worst found so far?
  /*
  计算 frameA 中的特征 shortindexA 和 frameB 中的特征 i 描述子之间的距离，
  如果距离小于当前存储和 frameB 特征最小距离的最大值，则根据大小关系将 i 放到最小距离存储 aiBest 中
  */
  tmpdist = matchingAlgorithm->distance(shortindexA, i);
  if (tmpdist < aiBest[numBest_ - 1].distance) {
    pairing_t tmp(static_cast<int>(i), tmpdist);
    // lb 是 shortindexA 和 i 的距离排在 aiBest 中的位置
    typename std::vector<pairing_t>::iterator lb = std::lower_bound(
        aiBest.begin(), aiBest.end(), tmp);  //get position for insertion
    typename std::vector<pairing_t>::iterator it, it_next;
    it = it_next = aiBest.end();

    --it;
    --it_next;
    // Insert the new match value into the list
    // 将 lb 之后的向后移动，将 lb 插入
    while (it_next != lb) {
      --it;
      *it_next = *it;  //move value one position to the back
      --it_next;
    }
    *lb = tmp;  //insert both index and score to the correct position to keep strict weak->strong ordering
  }
}

// The threading worker. This matches a keypoint with every other keypoint to find the best match.
template<typename MATCHING_ALGORITHM_T>
void DenseMatcher::doWorkLinearMatching(
    MatchJob & my_job, MATCHING_ALGORITHM_T * matchingAlgorithm) {
  OKVIS_ASSERT_TRUE(std::runtime_error, matchingAlgorithm != NULL,
                    "matching algorithm is NULL");
  try {
    // 几个线程一起做匹配 start 是线程的编号    
    int start = my_job.iThreadID;
    distance_t const_distthres = matchingAlgorithm->distanceThreshold();
    // useDistanceRatioThreshold_ 程序里实际没用    
    if (useDistanceRatioThreshold_) {
      // When using the distance ratio threshold, we want to build a list of good matches
      // independent of the threshold first and then later threshold on the ratio.
      // 如果用 useDistanceRatioThreshold_ 则 const_distthres 设到数值上限      
      const_distthres = std::numeric_limits<distance_t>::max();
    }

    size_t sizeA = matchingAlgorithm->sizeA();
    // 几个线程分开做匹配，所以 shortindexA += numMatcherThreads_
    // 对于 frameA 的特征遍历，在 frameB 中找匹配    
    for (size_t shortindexA = start; shortindexA < sizeA; shortindexA +=
        numMatcherThreads_) {
      // 如果 frameA 的特征在 setup 阶段设置了跳过          
      if (matchingAlgorithm->skipA(shortindexA))
        continue;

      //typename DenseMatcher::distance_t tmpdist;
      // vMyBest 是两层的 vector 第一层和 frameA 中特征的数目相同，第二层存储 frameA 中每个特征在 frameB 中的匹配点
      std::vector<pairing_t> & aiBest = (*my_job.vMyBest)[shortindexA];

      // initialize the best match to be -1 (no match) and set the score to be the distance threshold
      // No matches worse than the distance threshold will get through.
      aiBest.resize(numBest_, pairing_t(-1, const_distthres));  //the best x matches for this feature from the long list

      size_t numElementsInListB = matchingAlgorithm->sizeB();
      // 对于 frameA 中的一个特征和 frameB 中每个特征匹配计算描述子之间距离，
      // 作者说他用的描述子计算速度块，这样暴力匹配比 guided matching 速度还要快
      for (size_t i = 0; i < numElementsInListB; ++i) {
        if (matchingAlgorithm->skipB(i)) {
          continue;
        }
        // frameA 中的特征 shortindexA 和 frameB 中的特征 i 匹配，把匹配的结果放到 aiBest 中
        listBIteration(matchingAlgorithm, aiBest, shortindexA, i);

      }
      assignbest(static_cast<int>(shortindexA), *(my_job.vpairs),
                 *(my_job.vMyBest), my_job.mutexes, 0);  //this call assigns the match and reassigns losing matches recursively
    }
  } catch (const std::exception & e) {
    // \todo Install an error handler in the matching algorithm?
    std::cout << "\033[31mException in matching thread:\033[0m " << e.what();
  }
}

// The threading worker. This matches a keypoint with only a subset of the other keypoints
// to find the best match. (From matchingAlgorithm->getListBStartIterator() to
// MatchingAlgorithm->getListBEndIterator().
template<typename MATCHING_ALGORITHM_T>
void DenseMatcher::doWorkImageSpaceMatching(
    MatchJob & my_job, MATCHING_ALGORITHM_T* matchingAlgorithm) {
  OKVIS_ASSERT_TRUE(std::runtime_error, matchingAlgorithm != NULL,
                    "matching algorithm is NULL");
  try {
    int start = my_job.iThreadID;

    size_t numElementsInListB = matchingAlgorithm->sizeB();
    size_t numElementsInListA = matchingAlgorithm->sizeA();

    distance_t const_distthres = matchingAlgorithm->distanceThreshold();
    if (useDistanceRatioThreshold_) {
      // When using the distance ratio threshold, we want to build a list of good matches
      // independent of the threshold first and then later threshold on the ratio.
      const_distthres = std::numeric_limits<distance_t>::max();
    }

    for (size_t shortindexA = start; shortindexA < matchingAlgorithm->sizeA();
        shortindexA += numMatcherThreads_) {
      if (matchingAlgorithm->skipA(shortindexA))
        continue;

      typename DenseMatcher::distance_t tmpdist;
      std::vector<pairing_t>& aiBest = (*my_job.vMyBest)[shortindexA];

      // initialize the best match to be -1 (no match) and set the score to be the distance threshold
      // No matches worse than the distance threshold will get through.
      aiBest.resize(numBest_, pairing_t(-1, const_distthres));  //the best x matches for this feature from the long list

      typename MATCHING_ALGORITHM_T::listB_tree_structure_t::iterator itBegin =
          matchingAlgorithm->getListBStartIterator(shortindexA);
      typename MATCHING_ALGORITHM_T::listB_tree_structure_t::iterator itEnd =
          matchingAlgorithm->getListBEndIterator(shortindexA);
      //check all features from the long list
      for (typename MATCHING_ALGORITHM_T::listB_tree_structure_t::iterator it =
          itBegin; it != itEnd; ++it) {
        size_t i = it->second;

        if (matchingAlgorithm->skipB(i)) {
          continue;
        }

        listBIteration(matchingAlgorithm, aiBest, shortindexA, i);

      }

      assignbest(static_cast<int>(shortindexA), *(my_job.vpairs),
                 *(my_job.vMyBest), my_job.mutexes, 0);  //this call assigns the match and reassigns losing matches recursively
    }

  } catch (const std::exception & e) {
    // \todo Install an error handler in the matching algorithm?
    std::cout << "\033[31mException in matching thread:\033[0m " << e.what();
  }
}

}  // namespace okvis
