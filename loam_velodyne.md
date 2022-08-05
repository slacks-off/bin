# loam_velodyne
## Angle
### public
#### attribute
##### rad
##### deg
##### cos
##### sin
#### method
##### operator
### private
#### attribute
##### _radian
##### _cos
##### _sin

---

## BasicLaserMapping
### public
#### attribute
##### setScanPeriod
##### setMaxIterations
##### setDeltaTAbort
##### setDeltaRAbort
#### method
##### updateIMU
##### updateOdometry
##### laserCloud
##### laserCloudCornerLast
##### laserCloudSurfLast
##### downSizeFilterCorner
##### downSizeFilterSurf
##### downSizeFilterMap
##### frameCount
##### scanPeriod
##### maxIterations
##### deltaTAbort
##### deltaRAbort
##### transformAftMapped
##### transformBefMapped
##### laserCloudSurroundDS
### private
#### attribute
##### _laserOdometryTime
##### _scanPeriod
##### _stackFrameNum
##### _mapFrameNum
##### _frameCount
##### _mapFrameCount
##### _maxIterations
##### _deltaTAbort
##### _deltaRAbort
##### _laserCloudCen
##### _downsizedMapCreated
###### Width
###### Height
###### Depth
#### method
##### pcl::PointCloud<pcl::PointXYZI>::Ptr
###### _laserCloudCornerLast
###### _laserCloudSurfLast
###### _laserCloudFullRes
###### _laserCloudCornerStack
###### _laserCloudSurfStack
###### _laserCloudCornerStackDS
###### _laserCloudSurfStackDS
###### _laserCloudSurround
###### _laserCloudSurroundDS
###### _laserCloudCornerFromMap
###### _laserCloudSurfFromMap
###### _laserCloudOri
###### _coeffSel
##### std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>
###### _laserCloudCornerArray
###### _laserCloudSurfArray
###### _laserCloudCornerDSArray
###### _laserCloudSurfDSArray
##### pcl::VoxelGrid<pcl::PointXYZI>
###### _downSizeFilterCorner
###### _downSizeFilterSurf
###### _downSizeFilterMap

---

## BasicLaserOdometry
### public
#### attribute
##### setScanPeriod
##### setMaxIterations
##### setDeltaTAbort
##### setDeltaRAbort
#### method
##### process
##### updateIMU
##### cornerPointsSharp
##### cornerPointsLessSharp
##### surfPointsFlat
##### surfPointsLessFlat
##### laserCloud
##### transformSum
##### transform
##### lastCornerCloud
##### lastSurfaceCloud
##### frameCount
##### scanPeriod
##### maxIterations
##### deltaTAbort
##### deltaRAbort
##### transformToStart
##### pluginIMURotation
##### accumulateRotation
### private
#### attribute
##### _scanPeriod
##### _frameCount
##### _maxIterations
##### _systemInited
##### _deltaTAbort
##### _deltaRAbort
#### method
##### pcl::PointCloud<pcl::PointXYZI>::Ptr
###### _lastCornerCloud
###### _lastSurfaceCloud
###### _laserCloudOri
###### _coeffSel
###### _cornerPointsSharp
###### _cornerPointsLessSharp
###### _surfPointsFlat
###### _surfPointsLessFlat
###### _laserCloud
##### nanoflann::KdTreeFLANN<pcl::PointXYZI>
###### _lastCornerKDTree
###### _lastSurfaceKDTree
#####  std::vector<int>
###### _pointSearchCornerInd1
###### _pointSearchCornerInd2
###### _pointSearchSurfInd1
###### _pointSearchSurfInd2
###### _pointSearchSurfInd3
#### Twist
##### _transform
##### _transformSum
#### Angle
##### _imuRollStart
##### _imuPitchStart
##### _imuYawStart
##### _imuRollEnd
##### _imuPitchEnd
##### _imuYawEnd
#### Vector3
##### _imuShiftFromStart
##### _imuVeloFromStart

---

## BasicScanRegistration
### RegistrationParams
#### public
##### scanPeriod
##### imuHistorySize
##### nFeatureRegions
##### curvatureRegion
##### maxCornerSharp
##### maxCornerLessSharp
##### maxSurfaceFlat
##### lessFlatFilterSize
##### surfaceCurvatureThreshold
### typedef struct IMUState
#### Time
##### stamp
#### Angle
##### roll
##### pitch
##### yaw
#### Vector3
##### position
##### velocity
##### acceleration
#### method
##### interpolate
###### start
###### end
###### ratio
###### result
### BasicScanRegistration
#### public
##### attribute
###### configure
##### method
###### processScanlines
###### updateIMUData
###### projectPointToStartOfSweep
###### imuTransform
###### sweepStart
###### laserCloud
###### cornerPointsSharp
###### cornerPointsLessSharp
###### surfacePointsFlat
###### surfacePointsLessFlat
###### config
#### private attribute
##### pcl::PointCloud<pcl::PointXYZI>
###### _laserCloud
###### _cornerPointsSharp
###### _cornerPointsLessSharp
###### _surfacePointsFlat
###### _surfacePointsLessFlat
##### Time
###### _sweepStart
###### _scanTime
##### IMUState
###### _imuStart
###### _imuCur
##### Vector3
###### _imuPositionShift
##### size_t
###### _imuIdx 
##### CircularBuffer<IMUState>
###### _imuHistory
##### std::vector
###### _regionCurvature
###### _regionLabel
###### _regionSortIndices
###### _scanNeighborPicked
#### private method
##### hasIMUData
##### setIMUTransformFor
##### transformToStartIMU
##### reset
##### extractFeatures
##### setRegionBuffersFor
##### setScanBuffersFor
##### markAsPicked
##### interpolateIMUStateFor
##### updateIMUTransform


## BasicTransformMaintenance
### public
#### attribute
##### updateOdometry
##### updateMappingTransform
##### updateMappingTransform
#### method
##### transformAssociateToMap
##### transformMapped
### private
#### attribute
##### _transformSum
##### _transformIncre
##### _transformMapped
##### _transformBefMapped
##### _transformAftMapped

---

## CircularBuffer
### public
#### attribute
##### size
##### capacity
#### method
##### ensureCapacity
##### operator
##### first
##### empty
##### last
##### push
### private attribute
#### size_t
##### _capacity
##### _size
##### _startIdx
#### T*
##### _buffer

---

## common

---

## LaserMapping
### public
#### attribute
##### LaserMapping
##### setup
###### ros::NodeHandle& node
###### ros::NodeHandle& privateNode
##### laserCloudCornerLastHandler
##### laserCloudSurfLastHandler
##### laserCloudFullResHandler
##### laserOdometryHandler
##### imuHandler
##### spin
##### process
### protected
#### reset
#### hasNewData
#### publishResult
### private
#### attribute
##### ros::Time
###### _timeLaserCloudCornerLast
###### _timeLaserCloudSurfLast
###### _timeLaserCloudFullRes
###### _timeLaserOdometry
##### nav_msgs::Odometry
###### _odomAftMapped
##### tf::StampedTransform
###### _aftMappedTrans
##### ros::Publisher
###### _pubLaserCloudSurround
###### _pubLaserCloudFullRes
###### _pubOdomAftMapped
##### tf::TransformBroadcaster
###### _tfBroadcaster
##### ros::Subscriber
###### _subLaserCloudCornerLast
###### _subLaserCloudSurfLast
###### _subLaserCloudFullRes
###### _subLaserOdometry
###### _subImu

---

## LaserOdometry
### public
#### attribute
##### setup
###### ros::NodeHandle& node
###### ros::NodeHandle& privateNode
#### method
##### laserCloudSharpHandler
##### laserCloudLessSharpHandler
##### laserCloudFlatHandler
##### laserCloudLessFlatHandler
##### laserCloudFullResHandler
##### imuTransHandler
##### spin
##### process
### protected
#### reset
#### hasNewData
#### publishResult
### private attribute
#### ros::Time
##### _timeCornerPointsSharp
##### _timeCornerPointsLessSharp
##### _timeSurfPointsFlat
##### _timeSurfPointsLessFlat
##### _timeLaserCloudFullRes
##### _timeImuTrans
#### nav_msgs::Odometry
##### _laserOdometryMsg
#### tf::StampedTransform
##### _laserOdometryTrans
#### ros::Publisher
##### _pubLaserCloudCornerLast
##### _pubLaserCloudSurfLast
##### _pubLaserCloudFullRes
##### _pubLaserOdometry
#### tf::TransformBroadcaster
##### _tfBroadcaster
#### ros::Subscriber
##### _subCornerPointsSharp
##### _subCornerPointsLessSharp
##### _subSurfPointsFlat
##### _subSurfPointsLessFlat
##### _subLaserCloudFullRes
##### _subImuTrans
#### _newCornerPointsSharp
#### _newCornerPointsLessSharp
#### _newSurfPointsFlat
#### _newSurfPointsLessFlat
#### _newLaserCloudFullRes
#### _newImuTrans

---

## MultiScanRegistration
### public
#### MultiScanRegistration
#### setup
#### handleCloudMessage
### private
#### method
##### process
#### attribute
##### setupROS
## MultiScanMapper
### public
#### attribute
##### MultiScanMapper
##### getRingForAngle
#### method
##### getLowerBound
##### getUpperBound
##### getNumberOfScanRings
##### set
##### Velodyne_VLP_16
##### Velodyne_HDL_32
##### Velodyne_HDL_64E
### private
#### attribute
##### float _lowerBound
##### float _upperBound
##### uint16_t _nScanRings
##### float _factor

---

## RadiusResultSet
### public
#### attribute
##### clear
##### c
##### save_value
##### load_value
#### method
##### size
##### full
##### addPoint
##### worstDist
### std::pair<IndexType,DistanceType>
#### worst_item
## DataSource
### struct
#### L1_Adaptor 
#### L2_Adaptor
#### L2_Simple_Adaptor
#### SO2_Adaptor
##### evalMetric
##### accum_dist
#### SO3_Adaptor
#### metric_L1
#### metric_L2
#### metric_L2_Simple
#### metric_SO2
#### metric_SO3
#### KDTreeSingleIndexAdaptorParams
#### SearchParams
## PooledAllocator
### public
#### size_t
###### usedMemory
###### wastedMemory
#### PooledAllocator
##### internal_init
##### free_all
##### malloc
## CArray
### public
#### attribute
##### T
###### value_type
###### iterator
###### const_iterator
###### reference
###### const_reference
##### std::size_t
###### size_type
##### std::ptrdiff_t
###### difference_type
#### method
##### iterator
###### begin
###### end
##### const_iterator
###### begin
###### end
### private
#### rangecheck
## KDTreeBaseClass
### public
#### attribute
##### freeIndex
##### computeMinMax
##### divideTree
#### method
##### size
##### veclen
##### dataset_get
##### usedMemory
##### middleSplit_
##### planeSplit
##### computeInitialDistances
##### save_tree
##### load_tree
##### saveIndex_
##### loadIndex_
## KDTreeSingleIndexAdaptor
### public
#### attribute
##### init_vind
##### computeBoundingBox
#### method
##### KDTreeSingleIndexAdaptor
##### buildIndex
##### findNeighbors
### private method
#### KDTreeSingleIndexAdaptor
## RESULTSET
### public
#### attribute
##### searchLevel
##### saveIndex
##### loadIndex
##### KDTreeSingleIndexDynamicAdaptor
#### method
##### getAllIndices
##### addPoints
##### removePoint
##### kdtree_get_bbox
### private
#### attribute
##### First0Bit
##### init
#### method
##### KDTreeEigenMatrixAdaptor

---

## KdTreeFLANN
### public
#### attribute
##### boost::shared_ptr
###### <KdTreeFLANN<PointT> > Ptr
###### <const KdTreeFLANN<PointT> > ConstPtr
###### <std::vector<int> > IndicesPtr
###### <const std::vector<int> > IndicesConstPtr
##### pcl::PointCloud<PointT>
###### PointCloud
###### ::Ptr PointCloudPtr
###### ::ConstPtr PointCloudConstPtr
##### nearestKSearch
##### radiusSearch
#### method
##### makeShared
##### setInputCloud

---

## ScanRegistration
### public
#### attribute
##### setupROS
##### handleIMUMessage
#### private
##### parseParams
### private
#### ros::Subscriber
##### _subImu
#### ros::Publisher
##### _pubLaserCloud
##### _pubCornerPointsSharp
##### _pubCornerPointsLessSharp
##### _pubSurfPointsFlat
##### _pubSurfPointsLessFlat
##### _pubImuTrans

---

## KDTreeSingleIndexAdaptor
### public
#### attribute
#####
#####
#####
#### method
#####
#####
#####
### private
#### attribute
#####
#####
#####
#### method
#####
#####
#####

---

## time_utils

---

## TransformMaintenance
### public
#### method
##### TransformMaintenance
##### laserOdometryHandler
##### odomAftMappedHandler
### private
#### nav_msgs::
##### Odometry
###### _laserOdometry2
#### tf::
##### _laserOdometryTrans2
##### _tfBroadcaster2
#### ros::
##### _pubLaserOdometry2
##### _subLaserOdometry
##### _subOdomAftMapped

---

## Twist
### public
#### attribute
##### Twist
###### rot_x
###### rot_y
###### rot_z
###### pos
