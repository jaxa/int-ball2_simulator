

#include "coordinate_transform/coordinate_transform.h"

//------------------------------------------------------------------------------
// デフォルトコンストラクタ
gazebo::CoordinateTransform::CoordinateTransform() :
iss_cg_(gz::math::Vector3d::Zero),
jpm_pos_(gz::math::Vector3d::Zero),
jpm_euler_(gz::math::Vector3d::Zero),
jpm_qtn_(gz::math::Quaterniond(1.0, 0.0, 0.0, 0.0)),
ds_pos_(gz::math::Vector3d::Zero),
ds_euler_(gz::math::Vector3d::Zero),
ds_qtn_(gz::math::Quaterniond(1.0, 0.0, 0.0, 0.0))
{
}

//------------------------------------------------------------------------------
// JPM Pose, Docking Station Poseによるコンストラクタ
gazebo::CoordinateTransform::CoordinateTransform(
	const gz::math::Vector3d& iss_cg,
	const gz::math::Vector3d& jpm_pos, const gz::math::Vector3d& jpm_euler, 
	const gz::math::Vector3d& ds_pos,  const gz::math::Vector3d& ds_euler
) :
iss_cg_(iss_cg),
jpm_pos_(jpm_pos),
jpm_euler_(jpm_euler),
jpm_qtn_(gz::math::Quaterniond(jpm_euler)),
ds_pos_(ds_pos),
ds_euler_(ds_euler),
ds_qtn_(gz::math::Quaterniond(ds_euler))
{
}

//------------------------------------------------------------------------------
// デストラクタ
gazebo::CoordinateTransform::~CoordinateTransform() = default;

//------------------------------------------------------------------------------
// コピーコンストラクタ
gazebo::CoordinateTransform::CoordinateTransform(const CoordinateTransform&) = default;

//------------------------------------------------------------------------------
// コピー代入演算子
gazebo::CoordinateTransform&
gazebo::CoordinateTransform::operator=(const CoordinateTransform&) = default;

//------------------------------------------------------------------------------
// ムーブコンストラクタ
gazebo::CoordinateTransform::CoordinateTransform(CoordinateTransform&&) = default;

//------------------------------------------------------------------------------
// ムーブ代入演算子
gazebo::CoordinateTransform&
gazebo::CoordinateTransform::operator=(CoordinateTransform&&) = default;

//------------------------------------------------------------------------------
// メンバ変数の設定
void gazebo::CoordinateTransform::set(
	const gz::math::Vector3d& iss_cg,
	const gz::math::Vector3d& jpm_pos, const gz::math::Vector3d& jpm_euler, 
	const gz::math::Vector3d& ds_pos,  const gz::math::Vector3d& ds_euler
)
{
	iss_cg_    = iss_cg;
	jpm_pos_   = jpm_pos;
	jpm_euler_ = jpm_euler;
	jpm_qtn_   = gz::math::Quaterniond(jpm_euler);
	ds_pos_    = ds_pos;
	ds_euler_  = ds_euler;
	ds_qtn_    = gz::math::Quaterniond(ds_euler);
}

//------------------------------------------------------------------------------
// メンバ変数の設定
void gazebo::CoordinateTransform::set(
	const gz::math::Vector3d& iss_cg,
	const gz::math::Vector3d& jpm_pos, const gz::math::Vector3d& jpm_euler
)
{
	iss_cg_    = iss_cg;
	jpm_pos_   = jpm_pos;
	jpm_euler_ = jpm_euler;
	jpm_qtn_   = gz::math::Quaterniond(jpm_euler);
	ds_pos_    = gz::math::Vector3d::Zero;
	ds_euler_  = gz::math::Vector3d::Zero;
	ds_qtn_    = gz::math::Quaterniond(1.0, 0.0, 0.0, 0.0);
}

//------------------------------------------------------------------------------
// World座標系での位置をLVLH(ISS座標系)へ変換
gz::math::Vector3d gazebo::CoordinateTransform::getLvlhPosFromWrold(const gz::math::Vector3d& world_pos)
{
	return world_pos - iss_cg_;
}

//------------------------------------------------------------------------------
// LVLH(ISS座標系)での位置をWorld座標系へ変換
gz::math::Vector3d gazebo::CoordinateTransform::getWorldPosFromLvlh(const gz::math::Vector3d& lvlh_pos)
{
	return lvlh_pos + iss_cg_;
}

//------------------------------------------------------------------------------
// LVLH(ISS座標系)での位置をISS機体座標系へ変換
gz::math::Vector3d gazebo::CoordinateTransform::getIssbodyPosFromLvlh(const gz::math::Vector3d& lvlh_pos, const gz::math::Quaterniond& iss_qtn)
{
	return iss_qtn.RotateVectorReverse(lvlh_pos);
}

///------------------------------------------------------------------------------
// ISS機体座標系での位置をLVLH(ISS座標系)へ変換
gz::math::Vector3d gazebo::CoordinateTransform::getLvlhPosFromIssbody(const gz::math::Vector3d& iss_body_pos, const gz::math::Quaterniond& iss_qtn)
{
	return iss_qtn.RotateVector(iss_body_pos);
}
		
//------------------------------------------------------------------------------
// LVLH(ISS座標系)での速度をISS機体座標系へ変換
gz::math::Vector3d gazebo::CoordinateTransform::getIssbodyVelFromLvlh(
	const gz::math::Vector3d&    lvlh_pos, const gz::math::Vector3d& lvlh_vel, 
	const gz::math::Quaterniond& iss_qtn,  const gz::math::Vector3d& iss_w
)
{
	auto   iss_body_pos(this->getIssbodyPosFromLvlh(lvlh_pos, iss_qtn));

	return iss_qtn.RotateVectorReverse(lvlh_vel) - iss_w.Cross(iss_body_pos);
}

//------------------------------------------------------------------------------
//  LVLH(ISS座標系)での姿勢クォータニオンをISS機体座標系へ変換
gz::math::Quaterniond gazebo::CoordinateTransform::getIssbodyQtnFromLvlh(const gz::math::Quaterniond& ib2_qtn, const gz::math::Quaterniond& iss_qtn)
{
	return iss_qtn.Inverse() * ib2_qtn;
}

//------------------------------------------------------------------------------
//  ISS機体座標系での位置をJPM基準座標系へ変換
gz::math::Vector3d gazebo::CoordinateTransform::getJpmRefPosFromIssbody(const gz::math::Vector3d& iss_body_pos)
{
	return jpm_qtn_.RotateVectorReverse(iss_body_pos - (jpm_pos_ - iss_cg_));
}

//------------------------------------------------------------------------------
//  JPM基準座標系での位置をISS機体座標系へ変換
gz::math::Vector3d gazebo::CoordinateTransform::getIssbodyPosFromJpmRef(const gz::math::Vector3d& jpm_ref_pos)
{
	return jpm_qtn_.RotateVector(jpm_ref_pos) + (jpm_pos_ - iss_cg_);
}

//------------------------------------------------------------------------------
//  ISS機体座標系での速度をJPM基準座標系へ変換
gz::math::Vector3d gazebo::CoordinateTransform::getJpmRefVelFromIssbody(const gz::math::Vector3d& iss_body_vel)
{
	return jpm_qtn_.RotateVectorReverse(iss_body_vel);
}

//------------------------------------------------------------------------------
//  ISS機体座標系に対する姿勢クォータニオンをJPM基準座標系へ変換
gz::math::Quaterniond gazebo::CoordinateTransform::getJpmRefQtnFromIssbody(const gz::math::Quaterniond& iss_body_qtn)
{
	return jpm_qtn_.Inverse() * iss_body_qtn;
}

//------------------------------------------------------------------------------
//  JPM基準座標系での位置をドッキングステーション座標系へ変換
gz::math::Vector3d gazebo::CoordinateTransform::getDsPosFromJpmRef(const gz::math::Vector3d& jpm_ref_pos)
{
	return ds_qtn_.RotateVectorReverse(jpm_ref_pos - ds_pos_);
}

//------------------------------------------------------------------------------
//  JPM基準座標系での速度をドッキングステーション座標系へ変換
gz::math::Vector3d gazebo::CoordinateTransform::getDsVelFromJpmRef(const gz::math::Vector3d& jpm_ref_vel)
{
	return ds_qtn_.RotateVectorReverse(jpm_ref_vel);
}

//------------------------------------------------------------------------------
//  JPM基準座標系に対する姿勢クォータニオンをドッキングステーション座標系へ変換
gz::math::Quaterniond gazebo::CoordinateTransform::getDsQtnFromJpmRef(const gz::math::Quaterniond& jpm_ref_qtn)
{
	return ds_qtn_.Inverse() * jpm_ref_qtn;
}

//------------------------------------------------------------------------------
// World座標系での位置をJPM基準座標系へ変換
gz::math::Vector3d gazebo::CoordinateTransform::getJpmRefPosFromWorld(const gz::math::Vector3d& world_pos, const gz::math::Quaterniond& iss_qtn)
{
	gz::math::Vector3d     lvlh_pos     = this->getLvlhPosFromWrold(world_pos);
	gz::math::Vector3d     iss_body_pos = this->getIssbodyPosFromLvlh(lvlh_pos, iss_qtn);
	gz::math::Vector3d     jpm_ref_pos  = this->getJpmRefPosFromIssbody(iss_body_pos);

	return jpm_ref_pos;
}

//------------------------------------------------------------------------------
// World座標系での速度をJPM基準座標系へ変換
gz::math::Vector3d gazebo::CoordinateTransform::getJpmRefVelFromWorld(
	const gz::math::Vector3d&    world_pos, const gz::math::Vector3d& world_vel, 
	const gz::math::Quaterniond& iss_qtn,   const gz::math::Vector3d& iss_w
)
{
	gz::math::Vector3d     lvlh_vel     = world_vel;
	gz::math::Vector3d     lvlh_pos     = this->getLvlhPosFromWrold(world_pos);
	gz::math::Vector3d     iss_body_vel = this->getIssbodyVelFromLvlh(lvlh_pos, lvlh_vel, iss_qtn, iss_w);
	gz::math::Vector3d     jpm_ref_vel  = this->getJpmRefVelFromIssbody(iss_body_vel);

	return jpm_ref_vel;
}

//------------------------------------------------------------------------------
// World座標系に対する姿勢クォータニオンをJPM基準座標系へ変換
gz::math::Quaterniond gazebo::CoordinateTransform::getJpmRefQtnFromWorld(const gz::math::Quaterniond& ib2_qtn, const gz::math::Quaterniond& iss_qtn)
{
	gz::math::Quaterniond  iss_body_qtn = this->getIssbodyQtnFromLvlh(gz::math::Quaterniond(ib2_qtn.W(), ib2_qtn.X(), ib2_qtn.Y(), ib2_qtn.Z()), iss_qtn);
	gz::math::Quaterniond  jpm_ref_qtn  = this->getJpmRefQtnFromIssbody(iss_body_qtn);

	return jpm_ref_qtn;
}

//------------------------------------------------------------------------------
//  World座標系での位置をドッキングステーション座標系へ変換
gz::math::Vector3d gazebo::CoordinateTransform::getDsPosFromWorld(const gz::math::Vector3d& world_pos, const gz::math::Quaterniond& iss_qtn)
{
	gz::math::Vector3d     jpm_ref_pos = this->getJpmRefPosFromWorld(world_pos, iss_qtn);
	gz::math::Vector3d     ds_pos      = this->getDsPosFromJpmRef(jpm_ref_pos);

	return ds_pos;
}

//------------------------------------------------------------------------------
//  World座標系での速度をドッキングステーション座標系へ変換
gz::math::Vector3d gazebo::CoordinateTransform::getDsVelFromWorld(
	const gz::math::Vector3d&    world_pos, const gz::math::Vector3d& world_vel, 
	const gz::math::Quaterniond& iss_qtn,   const gz::math::Vector3d& iss_w
)
{
	gz::math::Vector3d     jpm_ref_vel = this->getJpmRefVelFromWorld(world_pos, world_vel, iss_qtn, iss_w);
	gz::math::Vector3d     ds_vel      = this->getDsVelFromJpmRef(jpm_ref_vel);

	return ds_vel;
}

//------------------------------------------------------------------------------
//  World座標系に対する姿勢クォータニオンをドッキングステーション座標系へ変換
gz::math::Quaterniond gazebo::CoordinateTransform::getDsQtnFromWorld(const gz::math::Quaterniond& ib2_qtn, const gz::math::Quaterniond& iss_qtn)
{
	gz::math::Quaterniond  jpm_ref_qtn = this->getJpmRefQtnFromWorld(ib2_qtn, iss_qtn);
	gz::math::Quaterniond  ds_qtn      = this->getDsQtnFromJpmRef(jpm_ref_qtn);

	return ds_qtn;
}

//------------------------------------------------------------------------------
//  JPM基準座標系での位置をWorld座標系へ変換
gz::math::Vector3d gazebo::CoordinateTransform::getWorldPosFromJpmRef(const gz::math::Vector3d& jpm_ref_pos, const gz::math::Quaterniond& iss_qtn)
{
	gz::math::Vector3d     iss_body_pos = this->getIssbodyPosFromJpmRef(jpm_ref_pos);
	gz::math::Vector3d     lvlh_pos     = this->getLvlhPosFromIssbody(iss_body_pos, iss_qtn);
	gz::math::Vector3d     world_pos    = this->getWorldPosFromLvlh(lvlh_pos);
	
	return world_pos;
}
// End Of File -----------------------------------------------------------------
