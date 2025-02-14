
#include "ctl/dtc.h"
#include "guidance_control_common/RangeChecker.h"

#include <cassert>
//------------------------------------------------------------------------------
// ファイルスコープ
namespace
{
    bool moave_unbaiased = false;
}

//------------------------------------------------------------------------------
// デフォルトコンストラクタ
Dtc::Dtc(const ros::NodeHandle& nh) :
    nh_(nh), status_(DETECT::NONE),dtc_sigmaup_started_(false),
    sigma_start_counter_(0),sigma_end_counter_(0),sigma_end_counter_colcheck_(0),from_sigup_counter_(0),
    keep_state_counter_(0)
{
    using namespace ib2_mss;

    /** 航法キューサイズ */
    static const std::string ROSPARAM_QUE_SIZE             ("/nav_que/size");
    int que_size       (0);
    nh_.getParam(ROSPARAM_QUE_SIZE             , que_size);
    static const RangeCheckerUI32 QUE_SIZE_RANGE
    (RangeCheckerUI32::TYPE::GE, 2, true);
    QUE_SIZE_RANGE.valid(que_size, ROSPARAM_QUE_SIZE);
    que_size_  = que_size;
    ROS_INFO("/nav_que/que_size                  : %d",que_size_);

    /** メンバ設定 */
    setMember();

    /** 標準偏差計算初期化 */
	moave_daccx_.reset(new ib2_mss::MovingAverage(que_size_));
	moave_daccy_.reset(new ib2_mss::MovingAverage(que_size_));
	moave_daccz_.reset(new ib2_mss::MovingAverage(que_size_));
	moave_dwx_.reset(new ib2_mss::MovingAverage(que_size_));    
	moave_dwy_.reset(new ib2_mss::MovingAverage(que_size_));    
	moave_dwz_.reset(new ib2_mss::MovingAverage(que_size_));    
}

//------------------------------------------------------------------------------
// デストラクタ
Dtc::~Dtc() = default;

//------------------------------------------------------------------------------
// メンバ設定
bool Dtc::setMember()
{
    using namespace ib2_mss;

    bool ret = true;

    // ドッキング検知パラメータ
    ret = ret && nh_.getParam("/docking_chk/tolerance_pos"   , tolerance_pos_);
    ret = ret && nh_.getParam("/docking_chk/tolerance_att"   , tolerance_att_);
    ret = ret && nh_.getParam("/docking_chk/docking_pos/x"   , docking_pos_.x());
    ret = ret && nh_.getParam("/docking_chk/docking_pos/y"   , docking_pos_.y());
    ret = ret && nh_.getParam("/docking_chk/docking_pos/z"   , docking_pos_.z());
    ret = ret && nh_.getParam("/docking_chk/keep_state_num"  , keep_state_num_);
    ret = ret && nh_.getParam("/docking_chk/sigma_start_jud_num", dc_sigma_start_jud_num_);
    ret = ret && nh_.getParam("/docking_chk/sigma_start_dacc"   , dc_sigma_start_dacc_);
    ret = ret && nh_.getParam("/docking_chk/sigma_start_drate"  , dc_sigma_start_drate_);
    ret = ret && nh_.getParam("/att_profile/rda/x"           , docking_q_.x());
    ret = ret && nh_.getParam("/att_profile/rda/y"           , docking_q_.y());
    ret = ret && nh_.getParam("/att_profile/rda/z"           , docking_q_.z());
    ret = ret && nh_.getParam("/att_profile/rda/w"           , docking_q_.w());
    
    ret = ret && RangeCheckerD::positive(tolerance_pos_    , true, "/docking_chk/tolerance_pos");
    ret = ret && RangeCheckerD::positive(tolerance_att_    , true, "/docking_chk/tolerance_att");
    ret = ret && RangeCheckerD::positive(docking_q_.norm() , true, "docking attitude quaternion magnitude");

    static const RangeCheckerUI32 JUD_NUM_RANGE
    (RangeCheckerUI32::TYPE::GE, 1, true);

    ret = ret && JUD_NUM_RANGE.valid(keep_state_num_,                  "/docking_chk/keep_state_num_");
    ret = ret && JUD_NUM_RANGE.valid(dc_sigma_start_jud_num_ ,         "/docking_chk/sigma_start_jud_num");
    ret = ret && RangeCheckerD::positive(dc_sigma_start_dacc_ , true,  "/docking_chk/sigma_start_dacc");
    ret = ret && RangeCheckerD::positive(dc_sigma_start_drate_, true,  "/docking_chk/sigma_start_drate");

    ROS_INFO("******** Set Parameters in dtc.cpp");
    ROS_INFO("/docking_chk/tolerance_pos         : %f",tolerance_pos_);
    ROS_INFO("/docking_chk/tolerance_att         : %f",tolerance_att_);
    ROS_INFO("/docking_chk/docking_pos/x         : %f",docking_pos_.x());
    ROS_INFO("/docking_chk/docking_pos/y         : %f",docking_pos_.y());
    ROS_INFO("/docking_chk/docking_pos/z         : %f",docking_pos_.z());
    ROS_INFO("/docking_chk/keep_state_num        : %d",keep_state_num_);
    ROS_INFO("/docking_chk/sigma_start_jud_num   : %d",dc_sigma_start_jud_num_);
    ROS_INFO("/docking_chk/sigma_start_dacc      : %f",dc_sigma_start_dacc_);
    ROS_INFO("/docking_chk/sigma_start_drate     : %f",dc_sigma_start_drate_);
    ROS_INFO("/att_profile/rda/x                 : %f",docking_q_.x());
    ROS_INFO("/att_profile/rda/y                 : %f",docking_q_.y());
    ROS_INFO("/att_profile/rda/z                 : %f",docking_q_.z());
    ROS_INFO("/att_profile/rda/w                 : %f",docking_q_.w());

    // 衝突・クルーキャプチャリリースパラメータ
    ret = ret && nh_.getParam("/colcaprel_chk/colcap_id_jud_num"  , colcap_id_jud_num_);
    ret = ret && nh_.getParam("/colcaprel_chk/sigma_start_jud_num", sigma_start_jud_num_);
    ret = ret && nh_.getParam("/colcaprel_chk/sigma_end_jud_num"  , sigma_end_jud_num_);
    ret = ret && nh_.getParam("/colcaprel_chk/sigma_start_dacc"   , sigma_start_dacc_);
    ret = ret && nh_.getParam("/colcaprel_chk/sigma_end_dacc"     , sigma_end_dacc_);
    ret = ret && nh_.getParam("/colcaprel_chk/sigma_start_drate"  , sigma_start_drate_);
    ret = ret && nh_.getParam("/colcaprel_chk/sigma_end_drate"    , sigma_end_drate_);

    ret = ret && JUD_NUM_RANGE.valid(colcap_id_jud_num_   , "/colcaprel_chk/colcap_id_jud_num");
    ret = ret && JUD_NUM_RANGE.valid(sigma_start_jud_num_ , "/colcaprel_chk/sigma_start_jud_num");
    ret = ret && JUD_NUM_RANGE.valid(sigma_end_jud_num_   , "/colcaprel_chk/sigma_end_jud_num");

    ret = ret && RangeCheckerD::positive(sigma_start_dacc_    , true, "/colcaprel_chk/sigma_start_dacc");
    ret = ret && RangeCheckerD::positive(sigma_end_dacc_      , true, "/colcaprel_chk/sigma_end_dacc");
    ret = ret && RangeCheckerD::positive(sigma_start_drate_   , true, "/colcaprel_chk/sigma_start_drate");
    ret = ret && RangeCheckerD::positive(sigma_end_drate_     , true, "/colcaprel_chk/sigma_end_drate");

    ROS_INFO("/colcaprel_chk/colcap_id_jud_num   : %d",colcap_id_jud_num_);
    ROS_INFO("/colcaprel_chk/sigma_start_jud_num : %d",sigma_start_jud_num_);
    ROS_INFO("/colcaprel_chk/sigma_end_jud_num   : %d",sigma_end_jud_num_);
    ROS_INFO("/colcaprel_chk/sigma_start_dacc    : %f",sigma_start_dacc_);
    ROS_INFO("/colcaprel_chk/sigma_end_dacc      : %f",sigma_end_dacc_);
    ROS_INFO("/colcaprel_chk/sigma_start_drate   : %f",sigma_start_drate_);
    ROS_INFO("/colcaprel_chk/sigma_end_drate     : %f",sigma_end_drate_);

    if(!ret)
        ROS_ERROR("Parameter Setting Error in dtc.cpp");

    return ret;
}

//------------------------------------------------------------------------------
// 検知処理
Dtc::DETECT Dtc::detection(const ib2_msgs::Navigation nav_stamp,
 const int32_t ctl_status) 
{
    // 航法値をキューに格納
    history(nav_stamp);

    // 判定 
    check(ctl_status);

    return status_;
}

//------------------------------------------------------------------------------
// ドッキング目標値の取得
ib2_msgs::Navigation Dtc::dockingTarget(const ros::Time& t) const
{
    ib2_msgs::Navigation o;
    o.pose.header.stamp = t;
    o.pose.pose.position.x = docking_pos_.x();
    o.pose.pose.position.y = docking_pos_.y();
    o.pose.pose.position.z = docking_pos_.z();
    o.pose.pose.orientation.x = docking_q_.x();
    o.pose.pose.orientation.y = docking_q_.y();
    o.pose.pose.orientation.z = docking_q_.z();
    o.pose.pose.orientation.w = docking_q_.w();
    return o;
}

//------------------------------------------------------------------------------
// 検知ステータスのクリア
void Dtc::clearStatus() 
{
    // ステータスクリア
    status_= DETECT::NONE; 
    
    // 判定処理の初期化
    dtc_sigmaup_started_  = false;
    sigma_start_counter_  = 0;
    sigma_end_counter_    = 0;
    sigma_end_counter_colcheck_ = 0;
    from_sigup_counter_   = 0;
    keep_state_counter_   = 0;

}

//------------------------------------------------------------------------------
// 判定
void Dtc::check(const int32_t ctl_status)
{
    auto std_ax(moave_daccx_->s(moave_unbaiased));
    auto std_ay(moave_daccy_->s(moave_unbaiased));
    auto std_az(moave_daccz_->s(moave_unbaiased));
    auto std_av = Eigen::Vector3d (std_ax, std_ay, std_az);

    auto std_wx(moave_dwx_->s(moave_unbaiased));
    auto std_wy(moave_dwy_->s(moave_unbaiased));
    auto std_wz(moave_dwz_->s(moave_unbaiased));
    auto std_wv = Eigen::Vector3d (std_wx, std_wy, std_wz);
    
    std_a_ = std_av.norm();
    std_w_ = std_wv.norm();

    if(ctl_status==ib2_msgs::CtlStatusType::MOVING_TO_RDP ||
       ctl_status==ib2_msgs::CtlStatusType::DOCKING_STAND_BY)
    {
        // ドッキングステーションとの接触判定
        if(!dtc_sigmaup_started_)
        {
            dtc_sigmaup_started_ = sigmaAscent(dc_sigma_start_dacc_,
                                               dc_sigma_start_drate_, 
                                               dc_sigma_start_jud_num_);
        }
        else
        {
            // ドッキング判定 (RDA/AIP後から実施)
            dockingCheck();
        }
    }
    else
    {
        // 衝突/クルーのキャプチャ判定
        if(!dtc_sigmaup_started_)
        {
            dtc_sigmaup_started_ = sigmaAscent(sigma_start_dacc_,
                                               sigma_start_drate_,
                                               sigma_start_jud_num_);
        }
        else
        {
            // クルーキャプチャ判定
            if(status_ == DETECT::DISTURBED)
            {
                if(crewCapCheck()) return;
            }
            
            // 衝突・クルーリリース判定
            // colrelCheck();
            collisionCheck();
            releaseCheck();
        }
    }
}

//------------------------------------------------------------------------------
// 航法暦
void Dtc::history(const ib2_msgs::Navigation nav_stamp)
{
	static bool init = true;

    // 航法前回値
    if(init) 
    {
        last_nav_stamp_ = nav_stamp;
        init = false;
    }
    auto pre_nav (last_nav_stamp_);
 
    // 加速度差分
    auto ac = Eigen::Vector3d (nav_stamp.a.x, nav_stamp.a.y, nav_stamp.a.z);
    auto ap = Eigen::Vector3d (pre_nav.a.x, pre_nav.a.y, pre_nav.a.z);
    auto dacc (ac-ap);
    moave_daccx_->push_back(dacc.x());
    moave_daccy_->push_back(dacc.y());
    moave_daccz_->push_back(dacc.z());

    // 角速度差分
    auto wc = Eigen::Vector3d (nav_stamp.twist.angular.x,
             nav_stamp.twist.angular.y, nav_stamp.twist.angular.z);
    auto wp = Eigen::Vector3d (pre_nav.twist.angular.x,
             pre_nav.twist.angular.y, pre_nav.twist.angular.z);
    auto dw = wc-wp;
    moave_dwx_->push_back(dw.x());
    moave_dwy_->push_back(dw.y());
    moave_dwz_->push_back(dw.z());

    // 最新の位置
    auto rc(nav_stamp.pose.pose.position);
    rc_ =  Eigen::Vector3d (rc.x, rc.y, rc.z);

    // 最新の姿勢
    auto qc(nav_stamp.pose.pose.orientation);
    qc_=  Eigen::Quaterniond (qc.w, qc.x, qc.y, qc.z); 

    // 航法値を保存
    last_nav_stamp_ = nav_stamp;
}

//------------------------------------------------------------------------------
// 標準偏差の立ち上がり判定
bool Dtc::sigmaAscent(const double sigma_dacc,const double sigma_drate,
  const int sigma_jud_num)
{
    // 標準偏差 > 閾値　判定
    if(std_a_ > sigma_dacc || std_w_ > sigma_drate) 
    {
        sigma_start_counter_++;
    }
    else
    {
        sigma_start_counter_ = 0;
    }

    // 閾値を超えた回数の判定
    if(sigma_start_counter_ >= sigma_jud_num)
    {
        sigma_start_counter_ = 0;
        status_  = DETECT::DISTURBED;
        ROS_INFO("DISTURBED");
        return true;
    }
    return false;
}

//------------------------------------------------------------------------------
// 衝突・クルーリリース判定
void Dtc::colrelCheck()
{
    // 標準偏差の下降チェック
    if(std_a_ < sigma_end_dacc_ && std_w_ < sigma_end_drate_) 
    {
        sigma_end_counter_++;
    }
    else
    {
        sigma_end_counter_ = 0;
    }
 
    // 衝突・クルーリリース判定
    if(sigma_end_counter_ >= sigma_end_jud_num_)
    {
        if(status_ == DETECT::CREW_CAPTURE)
        {
            // クルーリリース判定
            status_  = DETECT::CREW_RELEASE;
            ROS_INFO("RELEASED");
            sigma_end_counter_ = 0;
        }
        if(status_ == DETECT::DISTURBED)
        {
            // 衝突判定
            status_  = DETECT::COLLISION;
            ROS_INFO("COLLISION");
            sigma_end_counter_ = 0;
        }
    } 
}

//------------------------------------------------------------------------------
// 衝突判定
void Dtc::collisionCheck()
{
    // 標準偏差の下降チェック
    if(std_a_ < sigma_start_dacc_ && std_w_ < sigma_start_drate_) 
    {
        sigma_end_counter_colcheck_++;
    }
    else
    {
        sigma_end_counter_colcheck_ = 0;
    }
 
    if(sigma_end_counter_colcheck_ >= sigma_end_jud_num_)
    {
        if(status_ == DETECT::DISTURBED)
        {
            // 衝突判定
            status_  = DETECT::COLLISION;
            ROS_INFO("COLLISION");
            sigma_end_counter_colcheck_ = 0;
        }
    } 
}

//------------------------------------------------------------------------------
// クルーリリース判定
void Dtc::releaseCheck()
{
    // 標準偏差の下降チェック
    if(std_a_ < sigma_end_dacc_ && std_w_ < sigma_end_drate_) 
    {
        sigma_end_counter_++;
    }
    else
    {
        sigma_end_counter_ = 0;
    }
 
    if(sigma_end_counter_ >= sigma_end_jud_num_)
    {
        if(status_ == DETECT::CREW_CAPTURE)
        {
            // クルーリリース判定
            status_  = DETECT::CREW_RELEASE;
            ROS_INFO("RELEASED");
            sigma_end_counter_ = 0;
        }
    } 
}

//------------------------------------------------------------------------------
// クルーのキャプチャ判定
bool Dtc::crewCapCheck()
{
    from_sigup_counter_++;
    if(from_sigup_counter_ + sigma_start_jud_num_ >= colcap_id_jud_num_)
    {
        status_  = DETECT::CREW_CAPTURE;
        ROS_INFO("CAPTURED");
        from_sigup_counter_ = 0;
        return true;
    }
    return false;
}

//------------------------------------------------------------------------------
// ドッキング判定
void Dtc::dockingCheck()
{
    // 位置誤差
    auto dr(rc_ - docking_pos_);
    // 姿勢誤差
    Eigen::Quaterniond qe = docking_q_.conjugate()*qc_;
    double dq(2. * acos(qe.w())); //rad
    auto check_pos_att_num(keep_state_num_/2);

    // ドッキング位置保持判定
    if(keep_state_counter_ < keep_state_num_) 
    {
        // 位置・姿勢による判定
        if(keep_state_counter_ > check_pos_att_num && 
            (dr.norm() > tolerance_pos_ || fabs(dq) > tolerance_att_))
        {
            ROS_INFO("DOCKING NG: dr:%f,  dq:%f, counter:%d",dr.norm(),dq,keep_state_counter_); 
            keep_state_counter_ = 0;
            // 検知リセット
            dtc_sigmaup_started_ = false;
        }
        else
        {
            keep_state_counter_++;
        }
    }
    else
    {
        keep_state_counter_ = 0;
        status_ = DETECT::DOCKING;
        ROS_INFO("DOCKING OK");     
    }
}

//------------------------------------------------------------------------------
// ステータスの取得
Dtc::DETECT Dtc::status() const
{
    return status_;
}

// End Of File -----------------------------------------------------------------
