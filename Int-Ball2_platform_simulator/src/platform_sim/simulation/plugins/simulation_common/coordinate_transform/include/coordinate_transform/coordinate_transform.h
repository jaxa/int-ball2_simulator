
#pragma once
#include <gazebo/gazebo.hh>
//#include <ignition/math/Vector3.hh>

namespace gazebo
{
	/**
	 * @brief 座標変換.
	 *
	 * World座標系、ISS座標系、ISS機体座標系、JPM基準座標系、ホーム座標系間の座標変換を行う
	 */
	class CoordinateTransform final
	{
		//----------------------------------------------------------------------
		// コンストラクタ/デストラクタ
	public:
		/** デフォルトコンストラクタ */
		CoordinateTransform();

		/** ISS CG, JPM Pose, Docking Station Poseによるコンストラクタ
		 * @param [in] iss_cg    World座標系でのISS重心位置[m]
		 * @param [in] jpm_pos   ISSACSでのJPM基準点の位置(ISS機体に固定)[m]
		 * @param [in] jpm_euler ISSACSでのJPMの姿勢(ISS機体に固定)[rad]
		 * @param [in] ds_pos    JPM基準座標系でのドッキングステーションの原点[m]
		 * @param [in] ds_euler  JPM基準座標系でのドッキングステーションの姿勢[rad]
		 */
		CoordinateTransform(
			const ignition::math::Vector3d& iss_cg,
			const ignition::math::Vector3d& jpm_pos, const ignition::math::Vector3d& jpm_euler, 
			const ignition::math::Vector3d& ds_pos,  const ignition::math::Vector3d& ds_euler
		);
		
		/** デストラクタ. */
		~CoordinateTransform();
		
		//----------------------------------------------------------------------
		// コピー/ムーブ
	public:
		/** CoordinateTransform. */
		CoordinateTransform(const CoordinateTransform&);
		
		/** コピー代入演算子. */
		CoordinateTransform& operator=(const CoordinateTransform&);
		
		/** ムーブコンストラクタ. */
		CoordinateTransform(CoordinateTransform&&);
		
		/** ムーブ代入演算子. */
		CoordinateTransform& operator=(CoordinateTransform&&);

		//----------------------------------------------------------------------
		// 操作(Setter)
	public:
		/** メンバ変数の設定.
		 * @param [in] iss_cg    World座標系でのISS重心位置[m]
		 * @param [in] jpm_pos   ISSACSでのJPM基準点の位置(ISS機体に固定)[m]
		 * @param [in] jpm_euler ISSACSでのJPMの姿勢(ISS機体に固定)[rad]
		 * @param [in] ds_pos    JPM基準座標系でのドッキングステーションの原点[m]
		 * @param [in] ds_euler  JPM基準座標系でのドッキングステーションの姿勢[rad]
		 */
		void set
		(
			const ignition::math::Vector3d& iss_cg,
			const ignition::math::Vector3d& jpm_pos, const ignition::math::Vector3d& jpm_euler, 
			const ignition::math::Vector3d& ds_pos,  const ignition::math::Vector3d& ds_euler
		);

		/** メンバ変数の設定.
		 * @param [in] iss_cg    World座標系でのISS重心位置[m]
		 * @param [in] jpm_pos   ISSACSでのJPM基準点の位置(ISS機体に固定)[m]
		 * @param [in] jpm_euler ISSACSでのJPMの姿勢(ISS機体に固定)[rad]
		 */
		void set
		(
			const ignition::math::Vector3d& iss_cg,
			const ignition::math::Vector3d& jpm_pos, const ignition::math::Vector3d& jpm_euler
		);

		//----------------------------------------------------------------------
		// 実装
	public:
		/** World座標系での位置をLVLH(ISS座標系)へ変換
		 * @param [in] world_pos     World座標系での位置[m]
		 * @return                   LVLH(ISS座標系)での位置[m]
		 */
		ignition::math::Vector3d getLvlhPosFromWrold(const ignition::math::Vector3d& world_pos);

		/** LVLH(ISS座標系)での位置をISS機体座標系へ変換
		 * @param [in] lvlh_pos      LVLH(ISS座標系)での位置[m]
		 * @param [in] iss_qtn       LVLH(ISS座標系)に対するISS姿勢クォータニオン
		 * @return                   ISS機体座標系での位置[m]
		 */
		ignition::math::Vector3d getIssbodyPosFromLvlh(const ignition::math::Vector3d& lvlh_pos, const ignition::math::Quaterniond& iss_qtn);

		/** LVLH(ISS座標系)での速度をISS機体座標系へ変換
		 * @param [in] lvlh_pos      LVLH(ISS座標系)での位置[m]
		 * @param [in] lvlh_vel      LVLH(ISS座標系)での速度[m/s]
		 * @param [in] iss_qtn       LVLH(ISS座標系)に対するISS姿勢クォータニオン
		 * @param [in] iss_w         ISS姿勢角速度(ISS機体座標系)[rad]
		 * @return                   ISS機体座標系での速度[m/s]
		 */
		ignition::math::Vector3d getIssbodyVelFromLvlh(
			const ignition::math::Vector3d&    lvlh_pos, const ignition::math::Vector3d& lvlh_vel, 
			const ignition::math::Quaterniond& iss_qtn,  const ignition::math::Vector3d& iss_w
		);

		/** LVLH(ISS座標系)に対する姿勢クォータニオンをISS機体座標系へ変換
		 * @param [in] ib2_qtn       LVLH(ISS座標系)に対するIB2姿勢クォータニオン
		 * @param [in] iss_qtn       LVLH(ISS座標系)に対するISS姿勢クォータニオン
		 * @return                   ISS機体座標系に対する姿勢クォータニオン
		 */
		ignition::math::Quaterniond getIssbodyQtnFromLvlh(const ignition::math::Quaterniond& ib2_qtn, const ignition::math::Quaterniond& iss_qtn);

		/** ISS機体座標系での位置をJPM基準座標系へ変換
		 * @param [in] iss_body_pos  ISS機体座標系での位置[m]
		 * @return                   JPM基準座標系での位置[m]
		 */
		ignition::math::Vector3d getJpmRefPosFromIssbody(const ignition::math::Vector3d& iss_body_pos);
	
		/** ISS機体座標系での速度をJPM基準座標系へ変換
		 * @param [in] iss_body_vel  ISS機体座標系での速度[m/s]
		 * @return                   JPM基準座標系での速度[m/s]
		 */
		ignition::math::Vector3d getJpmRefVelFromIssbody(const ignition::math::Vector3d& iss_body_vel);

		/** ISS機体座標系に対する姿勢クォータニオンをJPM基準座標系へ変換
		 * @param [in] iss_body_qtn  ISS機体座標系に対する姿勢クォータニオン
		 * @return                   JPM基準座標系に対する姿勢クォータニオン
		 */
		ignition::math::Quaterniond getJpmRefQtnFromIssbody(const ignition::math::Quaterniond& iss_body_qtn);

		/** JPM基準座標系での位置をドッキングステーション座標系へ変換
		 * @param [in] jpm_ref_pos    JPM基準座標系での位置[m]
		 * @return                    ドッキングステーション座標系での位置[m]
		 */
		ignition::math::Vector3d getDsPosFromJpmRef(const ignition::math::Vector3d& jpm_ref_pos);

		/** JPM基準座標系での速度をドッキングステーション座標系へ変換
		 * @param [in] jpm_ref_vel    JPM基準座標系での速度[m/s]
		 * @return                    ドッキングステーション座標系での速度[m/s]
		 */
		ignition::math::Vector3d getDsVelFromJpmRef(const ignition::math::Vector3d& jpm_ref_vel);

		/** JPM基準座標系に対する姿勢クォータニオンをドッキングステーション座標系へ変換
		 * @param [in] jpm_ref_qtn    JPM基準座標系に対する姿勢クォータニオン
		 * @return                    ドッキングステーション座標系に対する姿勢クォータニオン
		 */
		ignition::math::Quaterniond getDsQtnFromJpmRef(const ignition::math::Quaterniond& jpm_ref_qtn);

		/** World座標系での位置をJPM基準座標系へ変換
		 * @param [in] world_pos     World座標系での位置[m]
		 * @param [in] iss_qtn       LVLH(ISS座標系)に対するISS姿勢クォータニオン
		 * @return                   JPM基準座標系での位置[m]
		 */
		ignition::math::Vector3d getJpmRefPosFromWorld(const ignition::math::Vector3d& world_pos, const ignition::math::Quaterniond& iss_qtn);

		/**  World座標系での速度をJPM基準座標系へ変換
		 * @param [in] world_pos     World座標系での位置[m]
		 * @param [in] world_vel     World座標系での速度[m/s]
		 * @param [in] iss_qtn       LVLH(ISS座標系)に対するISS姿勢クォータニオン
		 * @param [in] iss_w         ISS姿勢角速度(ISS機体座標系)[rad]
		 * @return                   JPM基準座標系での速度[m/s]
		 */
		ignition::math::Vector3d getJpmRefVelFromWorld(
			const ignition::math::Vector3d&    world_pos, const ignition::math::Vector3d& world_vel, 
			const ignition::math::Quaterniond& iss_qtn,   const ignition::math::Vector3d& iss_w
		);

		/** World座標系に対する姿勢クォータニオンをJPM基準座標系へ変換
		 * @param [in] ib2_qtn       World座標系に対するIB2姿勢クォータニオン
		 * @param [in] iss_qtn       LVLH(ISS座標系)に対するISS姿勢クォータニオン
		 * @return                   JPM基準座標系に対する姿勢クォータニオン
		 */
		ignition::math::Quaterniond getJpmRefQtnFromWorld(const ignition::math::Quaterniond& ib2_qtn, const ignition::math::Quaterniond& iss_qtn);

		/** World座標系での位置をドッキングステーション座標系へ変換
		 * @param [in] world_pos     World座標系での位置[m]
		 * @param [in] iss_qtn       LVLH(ISS座標系)に対するISS姿勢クォータニオン
		 * @return                  ドッキングステーション座標系での位置[m]
		 */
		ignition::math::Vector3d getDsPosFromWorld(const ignition::math::Vector3d& world_pos, const ignition::math::Quaterniond& iss_qtn);

		/**  World座標系での速度をドッキングステーション座標系へ変換
		 * @param [in] world_pos     World座標系での位置[m]
		 * @param [in] world_vel     World座標系での速度[m/s]
		 * @param [in] iss_qtn       LVLH(ISS座標系)に対するISS姿勢クォータニオン
		 * @param [in] iss_w         ISS姿勢角速度(ISS機体座標系)[rad]
		 * @return                   ドッキングステーション座標系での速度[m/s]
		 */
		ignition::math::Vector3d getDsVelFromWorld(
			const ignition::math::Vector3d&    world_pos, const ignition::math::Vector3d& world_vel, 
			const ignition::math::Quaterniond& iss_qtn,   const ignition::math::Vector3d& iss_w
		);

		/** World座標系に対する姿勢クォータニオンをドッキングステーション座標系へ変換
		 * @param [in] ib2_qtn       World座標系に対するIB2姿勢クォータニオン
		 * @param [in] iss_qtn       LVLH(ISS座標系)に対するISS姿勢クォータニオン
		 * @return                   ドッキングステーション座標系に対する姿勢クォータニオン
		 */
		ignition::math::Quaterniond getDsQtnFromWorld(const ignition::math::Quaterniond& ib2_qtn, const ignition::math::Quaterniond& iss_qtn);

		/** LVLH(ISS座標系)での位置をWorld座標系へ変換
		 * @param [in] lvlh_pos      LVLH(ISS座標系)での位置[m]
		 * @return                   World座標系での位置[m]
		 */
		ignition::math::Vector3d getWorldPosFromLvlh(const ignition::math::Vector3d& lvlh_pos);

		/** ISS機体座標系での位置をLVLH(ISS座標系)へ変換
		 * @param [in] iss_body_pos  ISS機体座標系での位置[m]
		 * @param [in] iss_qtn       LVLH(ISS座標系)に対するISS姿勢クォータニオン
		 * @return                   LVLH(ISS座標系)での位置[m]
		 */
		ignition::math::Vector3d getLvlhPosFromIssbody(const ignition::math::Vector3d& iss_body_pos, const ignition::math::Quaterniond& iss_qtn);

		/** JPM基準座標系での位置をISS機体座標系へ変換
		 * @param [in] jpm_ref_pos   JPM基準座標系での位置[m]
		 * @return                   ISS機体座標系での位置[m]
		 */
		ignition::math::Vector3d getIssbodyPosFromJpmRef(const ignition::math::Vector3d& jpm_ref_pos);
	
		/** JPM基準座標系での位置をWorld座標系へ変換
		 * @param [in] jpm_ref_pos   JPM基準座標系での位置[m]
		 * @param [in] iss_qtn       LVLH(ISS座標系)に対するISS姿勢クォータニオン
		 * @return                   World座標系での位置[m]
		 */
		ignition::math::Vector3d getWorldPosFromJpmRef(const ignition::math::Vector3d& jpm_ref_pos, const ignition::math::Quaterniond& iss_qtn);

		//----------------------------------------------------------------------
		// メンバー変数
	private:
		/** World座標系でのISS重心位置 */
		ignition::math::Vector3d          iss_cg_;

		/** ISS機体座標系でのJPM基準点の位置 */
		ignition::math::Vector3d          jpm_pos_;

		/** ISS機体座標系でのJPMの姿勢(Euler) */
		ignition::math::Vector3d          jpm_euler_;

		/** ISS機体座標系でのJPMの姿勢(Quaternion) */
		ignition::math::Quaterniond       jpm_qtn_;

		/** JPM基準座標系でのドッキングステーション(ホーム座標系)原点 */
		ignition::math::Vector3d          ds_pos_;

		/** JPM基準座標系でのドッキングステーション姿勢(Euler) */
		ignition::math::Vector3d          ds_euler_;

		/** JPM基準座標系でのドッキングステーション姿勢(Quaternion) */
		ignition::math::Quaterniond       ds_qtn_;
	};
}

// End Of File -----------------------------------------------------------------
