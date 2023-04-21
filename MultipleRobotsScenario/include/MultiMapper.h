/** 
 * File:   MultiMapper.h
 * Author: hdaoud
 *
 * Created on November 15, 2016, 12:15 PM
 */

#ifndef MULTIMAPPER_H
#define MULTIMAPPER_H

#include <string>
#include "System.h"
#include "Map.h"
#include "Frame.h"
#include "KeyFrameDatabase.h"
#include "ORBVocabulary.h"
#include "ORBextractor.h"
#include "LoopClosing.h"
#include "LocalMapping.h"
#include "Viewer.h"
#include "MapDrawer.h"
#include "FrameDrawer.h"
#include "Converter.h"
#include <iomanip>

#include <set>
#include <mutex>

namespace iORB_SLAM {
class Map;
class LoopClosing;
class LocalMapping;
class KeyFrameDatabase;

class MultiMapper {
public:
    typedef pair<Map*, KeyFrameDatabase*> MapAndKFDB;
    typedef map<Map*, LoopClosing*, std::less<Map*>,
        Eigen::aligned_allocator<std::pair<Map* const, LoopClosing*> > > MapAndLoopCloser;
    typedef pair<Map*, Tracking*> MapAndTracker;
    typedef map<Map*, LocalMapping*, std::less<Map*>,
        Eigen::aligned_allocator<std::pair<Map* const, LocalMapping*> > > MapAndLocalMapper;
    typedef map<KeyFrame*, g2o::Sim3, std::less<KeyFrame*>,
        Eigen::aligned_allocator<std::pair<KeyFrame* const, g2o::Sim3> > > KeyFrameAndPose;

public:
    MultiMapper();
    MultiMapper(Map* pMap, KeyFrameDatabase* pDB, ORBVocabulary* pVoc);

    // Main function
    void Run();

    bool InitFromFile(const string& dirName);
    void AddMap(Map* mpMap, KeyFrameDatabase* mpKeyFrameDB); // Add completed map to the MultiMapper
    void AddInitMap(Map* mpMap, KeyFrameDatabase* mpKeyFrameDB);
    void EraseMap(Map* mpMap); // Remove specific Map from the MultiMapper
    int RequestMapUpdate(); // Request update of uncompleted maps from all robots/maps
    void RegisterMap(int mpID); //Register map/robot in the MultiMapper
    void UpdatePosesAndAdd(Map* pMap, Map* pMapBase, const g2o::Sim3 g2oScw, KeyFrame* pKFini); //Update Poses of Keyframes in the map where one of its keyframes matches a keyframe in the main map

    void SetLoopCloser(Map* pMap, LoopClosing* pLoopCloser);
    //    LoopClosing* GetLoopCloser();

    void SetTracker(Map* pMap, Tracking* pTracker);

    void SetLocalMapper(Map* pMap, LocalMapping* pLocalMapper);

    void SetVocabulary(ORBVocabulary* pVoc);

    void RequestReset();
    void RequestFinish(int mpMapCount);
    bool isFinished();
    bool GetmbUpdatingMapPoses();

    void SaveTrajectory(const string& filename);
    void SaveMapTrajectory(Map* pMap, const string& filename);

    void clear();

    vector<Map*> mvpMapOrigins;
    std::mutex mMutexMuliMapUpdate;

protected:
    bool hasCandidates();

    bool DetectLoop();

    void SearchAndFuse(const KeyFrameAndPose& CorrectedPosesMap, Map* pMap);

    void MergeMaps(Map* pMap, Map* pMapBase);

    void ResetIfRequested();
    bool mbResetRequested;
    std::mutex mMutexReset;

    //MultiMapper will close when all SLAM systems request to
    int mnFinishRequests;

    std::vector<MapAndKFDB> mvMapAndKFDB;
    unsigned int mnMapCount;

    MapAndLocalMapper mMapsLocalMappers;
    MapAndLoopCloser mMapsLoopClosers;
    std::vector<MapAndTracker> mvMapsTrackers;

    bool CheckFinish();
    void SetFinish();
    bool mbFinished;
    std::mutex mMutexFinish;
    bool mbUpdatingMapPoses;

    std::mutex mMutexMapAndKFDB;

    bool mbMatchedBefore;
    bool mbSwapped;
    cv::Mat Tcc;
    KeyFrame* mpMatchedKF;
    cv::Mat mScw;
    g2o::Sim3 mg2oScw;
    std::vector<MapPoint*> mvpCurrentMatchedPoints;
    std::vector<MapPoint*> mvpLoopMapPoints;

    //BoW
    ORBVocabulary* mpORBVocabulary;

    //System* mpiORBSystem;

    std::mutex mMutexMultiMapper;
};
}
#endif /* MULTIMAPPER_H */
