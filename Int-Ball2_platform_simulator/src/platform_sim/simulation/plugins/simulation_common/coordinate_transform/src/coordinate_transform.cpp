

#include "coordinate_transform/coordinate_transform.h"

//------------------------------------------------------------------------------
// デフォルトコンストラクタ
gazebo::CoordinateTransform::CoordinateTransform() :
iss_cg_(ignition::math::Vector3d::Zero),
jpm_pos_(ignition::math::Vector3d::Zero),
jpm_euler_(ignition::math::Vector3d::Zero),
jpm_qtn_(ignition::math::Quaterniond(1.0, 0.0, 0.0, 0.0)),
ds_pos_(ignition::math::Vector3d::Zero),
ds_euler_(ignition::math::Vector3d::Zero),
ds_qtn_(ignition::math::Quaterniond(1.0, 0.0, 0.0, 0.0))
{
}

//------------------------------------------------------------------------------
// JPM Pose, Docking Station Poseによるコンストラクタ
gazebo::CoordinateTransform::CoordinateTransform(
	const ignition::math::Vector3d& iss_cg,
	const ignition::math::Vector3d& jpm_pos, const ignition::math::Vector3d& jpm_euler, 
	const ignition::math::Vector3d& ds_pos,  const ignition::math::Vector3d& ds_euler
) :
iss_cg_(iss_cg),
jpm_pos_(jpm_pos),
jpm_euler_(jpm_euler),
jpm_qtn_(ignition::math::Quaterniond(jpm_euler)),
ds_pos_(ds_pos),
ds_euler_(ds_euler),
ds_qtn_(ignition::math::Quaterniond(ds_euler))
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
	const ignition::math::Vector3d& iss_cg,
	const ignition::math::Vector3d& jpm_pos, const ignition::math::Vector3d& jpm_euler, 
	const ignition::math::Vector3d& ds_pos,  const ignition::math::Vector3d& ds_euler
)
{
	iss_cg_    = iss_cg;
	jpm_pos_   = jpm_pos;
	jpm_euler_ = jpm_euler;
	jpm_qtn_   = ignition::math::Quaterniond(jpm_euler);
	ds_pos_    = ds_pos;
	ds_euler_  = ds_euler;
	ds_qtn_    = ignition::math::Quaterniond(ds_euler);
}

//------------------------------------------------------------------------------
// メンバ変数の設定
void gazebo::CoordinateTransform::set(
	const ignition::math::Vector3d& iss_cg,
	const ignition::math::Vector3d& jpm_pos, const ignition::math::Vector3d& jpm_euler
)
{
	iss_cg_    = iss_cg;
	jpm_pos_   = jpm_pos;
	jpm_euler_ = jpm_euler;
	jpm_qtn_   = ignition::math::Quaterniond(jpm_euler);
	ds_pos_    = ignition::math::Vector3d::Zero;
	ds_euler_  = ignition::math::Vector3d::Zero;
	ds_qtn_    = ignition::math::Quaterniond(1.0, 0.0, 0.0, 0.0);
}

//------------------------------------------------------------------------------
// World座標系での位置をLVLH(ISS座標系)へ変換
ignition::math::Vector3d gazebo::CoordinateTransform::getLvlhPosFromWrold(const ignition::math::Vector3d& world_pos)
{
	return world_pos - iss_cg_;
}

//------------------------------------------------------------------------------
// LVLH(ISS座標系)での位置をWorld座標系へ変換
ignition::math::Vector3d gazebo::CoordinateTransform::getWorldPosFromLvlh(const ignition::math::Vector3d& lvlh_pos)
{
	return lvlh_pos + iss_cg_;
}

//------------------------------------------------------------------------------
// LVLH(ISS座標系)での位置をISS機体座標系へ変換
ignition::math::Vector3d gazebo::CoordinateTransform::getIssbodyPosFromLvlh(const ignition::math::Vector3d& lvlh_pos, const ignition::math::Quaterniond& iss_qtn)
{
	return iss_qtn.RotateVectorReverse(lvlh_pos);
}

///------------------------------------------------------------------------------
// ISS機体座標系での位置をLVLH(ISS座標系)へ変換
ignition::math::Vector3d gazebo::CoordinateTransform::getLvlhPosFromIssbody(const ignition::math::Vector3d& iss_body_pos, const ignition::math::Quaterniond& iss_qtn)
{
	return iss_qtn.RotateVector(iss_body_pos);
}
		
//------------------------------------------------------------------------------
// LVLH(ISS座標系)での速度をISS機体座標系へ変換
ignition::math::Vector3d gazebo::CoordinateTransform::getIssbodyVelFromLvlh(
	const ignition::math::Vector3d&    lvlh_pos, const ignition::math::Vector3d& lvlh_vel, 
	const ignition::math::Quaterniond& iss_qtn,  const ignition::math::Vector3d& iss_w
)
{
	auto   iss_body_pos(this->getIssbodyPosFromLvlh(lvlh_pos, iss_qtn));

	return iss_qtn.RotateVectorReverse(lvlh_vel) - iss_w.Cross(iss_body_pos);
}

//------------------------------------------------------------------------------
//  LVLH(ISS座標系)での姿勢クォータニオンをISS機体座標系へ変換
ignition::math::Quaterniond gazebo::CoordinateTransform::getIssbodyQtnFromLvlh(const ignition::math::Quaterniond& ib2_qtn, const ignition::math::Quaterniond& iss_qtn)
{
	return iss_qtn.Inverse() * ib2_qtn;
}

//------------------------------------------------------------------------------
//  ISS機体座標系での位置をJPM基準座標系へ変換
ignition::math::Vector3d gazebo::CoordinateTransform::getJpmRefPosFromIssbody(const ignition::math::Vector3d& iss_body_pos)
{
	return jpm_qtn_.RotateVectorReverse(iss_body_pos - (jpm_pos_ - iss_cg_));
}

//------------------------------------------------------------------------------
//  JPM基準座標系での位置をISS機体座標系へ変換
ignition::math::Vector3d gazebo::CoordinateTransform::getIssbodyPosFromJpmRef(const ignition::math::Vector3d& jpm_ref_pos)
{
	return jpm_qtn_.RotateVector(jpm_ref_pos) + (jpm_pos_ - iss_cg_);
}

//------------------------------------------------------------------------------
//  ISS機体座標系での速度をJPM基準座標系へ変換
ignition::math::Vector3d gazebo::CoordinateTransform::getJpmRefVelFromIssbody(const ignition::math::Vector3d& iss_body_vel)
{
	return jpm_qtn_.RotateVectorReverse(iss_body_vel);
}

//------------------------------------------------------------------------------
//  ISS機体座標系に対する姿勢クォータニオンをJPM基準座標系へ変換
ignition::math::Quaterniond gazebo::CoordinateTransform::getJpmRefQtnFromIssbody(const ignition::math::Quaterniond& iss_body_qtn)
{
	return jpm_qtn_.Inverse() * iss_body_qtn;
}

//------------------------------------------------------------------------------
//  JPM基準座標系での位置をドッキングステーション座標系へ変換
ignition::math::Vector3d gazebo::CoordinateTransform::getDsPosFromJpmRef(const ignition::math::Vector3d& jpm_ref_pos)
{
	return ds_qtn_.RotateVectorReverse(jpm_ref_pos - ds_pos_);
}

//------------------------------------------------------------------------------
//  JPM基準座標系での速度をドッキングステーション座標系へ変換
ignition::math::Vector3d gazebo::CoordinateTransform::getDsVelFromJpmRef(const ignition::math::Vector3d& jpm_ref_vel)
{
	return ds_qtn_.RotateVectorReverse(jpm_ref_vel);
}

//------------------------------------------------------------------------------
//  JPM基準座標系に対する姿勢クォータニオンをドッキングステーション座標系へ変換
ignition::math::Quaterniond gazebo::CoordinateTransform::getDsQtnFromJpmRef(const ignition::math::Quaterniond& jpm_ref_qtn)
{
	return ds_qtn_.Inverse() * jpm_ref_qtn;
}

//------------------------------------------------------------------------------
// World座標系での位置をJPM基準座標系へ変換
ignition::math::Vector3d gazebo::CoordinateTransform::getJpmRefPosFromWorld(const ignition::math::Vector3d& world_pos, const ignition::math::Quaterniond& iss_qtn)
{
	ignition::math::Vector3d     lvlh_pos     = this->getLvlhPosFromWrold(world_pos);
	ignition::math::Vector3d     iss_body_pos = this->getIssbodyPosFromLvlh(lvlh_pos, iss_qtn);
	ignition::math::Vector3d     jpm_ref_pos  = this->getJpmRefPosFromIssbody(iss_body_pos);

	return jpm_ref_pos;
}

//------------------------------------------------------------------------------
// World座標系での速度をJPM基準座標系へ変換
ignition::math::Vector3d gazebo::CoordinateTransform::getJpmRefVelFromWorld(
	const ignition::math::Vector3d&    world_pos, const ignition::math::Vector3d& world_vel, 
	const ignition::math::Quaterniond& iss_qtn,   const ignition::math::Vector3d& iss_w
)
{
	ignition::math::Vector3d     lvlh_vel     = world_vel;
	ignition::math::Vector3d     lvlh_pos     = this->getLvlhPosFromWrold(world_pos);
	ignition::math::Vector3d     iss_body_vel = this->getIssbodyVelFromLvlh(lvlh_pos, lvlh_vel, iss_qtn, iss_w);
	ignition::math::Vector3d     jpm_ref_vel  = this->getJpmRefVelFromIssbody(iss_body_vel);

	return jpm_ref_vel;
}

//------------------------------------------------------------------------------
// World座標系に対する姿勢クォータニオンをJPM基準座標系へ変換
ignition::math::Quaterniond gazebo::CoordinateTransform::getJpmRefQtnFromWorld(const ignition::math::Quaterniond& ib2_qtn, const ignition::math::Quaterniond& iss_qtn)
{
	ignition::math::Quaterniond  iss_body_qtn = this->getIssbodyQtnFromLvlh(ignition::math::Quaterniond(ib2_qtn.W(), ib2_qtn.X(), ib2_qtn.Y(), ib2_qtn.Z()), iss_qtn);
	ignition::math::Quaterniond  jpm_ref_qtn  = this->getJpmRefQtnFromIssbody(iss_body_qtn);

	return jpm_ref_qtn;
}

//------------------------------------------------------------------------------
//  World座標系での位置をドッキングステーション座標系へ変換
ignition::math::Vector3d gazebo::CoordinateTransform::getDsPosFromWorld(const ignition::math::Vector3d& world_pos, const ignition::math::Quaterniond& iss_qtn)
{
	ignition::math::Vector3d     jpm_ref_pos = this->getJpmRefPosFromWorld(world_pos, iss_qtn);
	ignition::math::Vector3d     ds_pos      = this->getDsPosFromJpmRef(jpm_ref_pos);

	return ds_pos;
}

//------------------------------------------------------------------------------
//  World座標系での速度をドッキングステーション座標系へ変換
ignition::math::Vector3d gazebo::CoordinateTransform::getDsVelFromWorld(
	const ignition::math::Vector3d&    world_pos, const ignition::math::Vector3d& world_vel, 
	const ignition::math::Quaterniond& iss_qtn,   const ignition::math::Vector3d& iss_w
)
{
	ignition::math::Vector3d     jpm_ref_vel = this->getJpmRefVelFromWorld(world_pos, world_vel, iss_qtn, iss_w);
	ignition::math::Vector3d     ds_vel      = this->getDsVelFromJpmRef(jpm_ref_vel);

	return ds_vel;
}

//------------------------------------------------------------------------------
//  World座標系に対する姿勢クォータニオンをドッキングステーション座標系へ変換
ignition::math::Quaterniond gazebo::CoordinateTransform::getDsQtnFromWorld(const ignition::math::Quaterniond& ib2_qtn, const ignition::math::Quaterniond& iss_qtn)
{
	ignition::math::Quaterniond  jpm_ref_qtn = this->getJpmRefQtnFromWorld(ib2_qtn, iss_qtn);
	ignition::math::Quaterniond  ds_qtn      = this->getDsQtnFromJpmRef(jpm_ref_qtn);

	return ds_qtn;
}

//------------------------------------------------------------------------------
//  JPM基準座標系での位置をWorld座標系へ変換
ignition::math::Vector3d gazebo::CoordinateTransform::getWorldPosFromJpmRef(const ignition::math::Vector3d& jpm_ref_pos, const ignition::math::Quaterniond& iss_qtn)
{
	ignition::math::Vector3d     iss_body_pos = this->getIssbodyPosFromJpmRef(jpm_ref_pos);
	ignition::math::Vector3d     lvlh_pos     = this->getLvlhPosFromIssbody(iss_body_pos, iss_qtn);
	ignition::math::Vector3d     world_pos    = this->getWorldPosFromLvlh(lvlh_pos);
	
	return world_pos;
}
// End Of File -----------------------------------------------------------------
