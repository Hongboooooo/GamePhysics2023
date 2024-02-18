#include "RigidBodySystemSimulator.h"
#include "collisionDetect.h"


RigidBodySystemSimulator::RigidBodySystemSimulator()
{
	m_iTestCase = 0;
}

const char* RigidBodySystemSimulator::getTestCasesStr() 
{
	return "Teapot,Random Objects,Triangle";
}

void RigidBodySystemSimulator::initUI(DrawingUtilitiesClass* DUC)
{
	this->DUC = DUC;
	switch (m_iTestCase)
	{
	case 0:break;
	case 1:
		//TwAddVarRW(DUC->g_pTweakBar, "Num Spheres", TW_TYPE_INT32, &m_iNumSpheres, "min=1");
		//TwAddVarRW(DUC->g_pTweakBar, "Sphere Size", TW_TYPE_FLOAT, &m_fSphereSize, "min=0.01 step=0.01");
		break;
	case 2:break;
	default:break;
	}

	addRigidBody(Vec3(3, 0, 0), Vec3(0.4, 0.6, 0.5), 1);
	addRigidBody(Vec3(-1, 0, 0), Vec3(0.2, 0.2, 0.2), 5);
	addRigidBody(Vec3(1, 0, 0), Vec3(0.2, 0.2, 0.2), 5);
	addRigidBody(Vec3(0, 0, 0), Vec3(0.4, 0.2, 1), 20);
	
	setOrientationOf(0, Quat(0.5, 0, 0.5, 0.5));
	setOrientationOf(1, Quat(0, 0.5, 0.5, 0.5));
	setOrientationOf(2, Quat(0, 0.5, 0.5, 0.5));

	applyForceOnBody(0, Vec3(2, 0, 0), Vec3(-1, 0, 0));
	applyForceOnBody(1, Vec3(-1, 0, 0), Vec3(2, 0, 0));
	applyForceOnBody(2, Vec3(1, 0, 0), Vec3(-2, 0, 0));


	
	

}

void RigidBodySystemSimulator::reset()
{
	m_mouse.x = m_mouse.y = 0;
	m_trackmouse.x = m_trackmouse.y = 0;
	m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
}

void RigidBodySystemSimulator::DrawRigidBodys()
{
	for (int idx = 0; idx < rigid_body_list.size(); idx++)
	{
		Mat4 scal_m = Mat4(
			rigid_body_list[idx].size.x, 0, 0, 0,
			0, rigid_body_list[idx].size.y, 0, 0,
			0, 0, rigid_body_list[idx].size.z, 0,
			0, 0, 0, 1
		);

		Mat4 rota_m = rigid_body_list[idx].orientation.getRotMat().inverse();

		Mat4 tran_m = Mat4(
			1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, 0,
			rigid_body_list[idx].position_mass_center.x, rigid_body_list[idx].position_mass_center.y, rigid_body_list[idx].position_mass_center.z, 1
		);

		DUC->drawRigidBody(scal_m * rota_m * tran_m);

		//for (int i = 0; i < rigid_body_list[idx].mass_point_list.size(); i++)
		//{
		//	DUC->drawSphere(rigid_body_list[idx].mass_point_list[i].position, Vec3(0.1, 0.1, 0.1));
		//}
	}
	
}

//void RigidBodySystemSimulator::drawMovableTeapot()
//{
//	DUC->setUpLighting(Vec3(), 0.4 * Vec3(1, 1, 1), 100, 0.6 * Vec3(0.97, 0.86, 1));
//	DUC->drawTeapot(m_vfMovableObjectPos, m_vfRotate, Vec3(0.5, 0.5, 0.5));
//}

void RigidBodySystemSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	//switch (m_iTestCase)
	//{
	//case 0: DrawRigidBodys(); break;
	////case 1: drawSomeRandomObjects(); break;
	////case 2: drawTriangle(); break;
	//}
	DrawRigidBodys();
	
}

void RigidBodySystemSimulator::notifyCaseChanged(int testCase)
{
	//m_iTestCase = testCase;
	//switch (m_iTestCase)
	//{
	//case 0:
	//	cout << "Teapot !\n";
	//	//m_vfMovableObjectPos = Vec3(0, 0, 0);
	//	//m_vfRotate = Vec3(0, 0, 0);
	//	break;
	//case 1:
	//	cout << "Random Object!\n";
	//	//m_iNumSpheres = 100;
	//	//m_fSphereSize = 0.05f;
	//	break;
	//case 2:
	//	cout << "Triangle !\n";
	//	break;
	//default:
	//	cout << "Empty Test!\n";
	//	break;
	//}
}

void RigidBodySystemSimulator::externalForcesCalculations(float timeElapsed)
{
	//don't have to exist
	// Apply the mouse deltas to g_vfMovableObjectPos (move along cameras view plane)
	//Point2D mouseDiff;
	//mouseDiff.x = m_trackmouse.x - m_oldtrackmouse.x;
	//mouseDiff.y = m_trackmouse.y - m_oldtrackmouse.y;
	//if (mouseDiff.x != 0 || mouseDiff.y != 0)
	//{
	//	Mat4 worldViewInv = Mat4(DUC->g_camera.GetWorldMatrix() * DUC->g_camera.GetViewMatrix());
	//	worldViewInv = worldViewInv.inverse();
	//	Vec3 inputView = Vec3((float)mouseDiff.x, (float)-mouseDiff.y, 0);
	//	Vec3 inputWorld = worldViewInv.transformVectorNormal(inputView);
	//	// find a proper scale!
	//	float inputScale = 0.001f;
	//	inputWorld = inputWorld * inputScale;
	//	m_vfMovableObjectPos = m_vfMovableObjectFinalPos + inputWorld;
	//}
	//else {
	//	m_vfMovableObjectFinalPos = m_vfMovableObjectPos;
	//}
}

void RigidBodySystemSimulator::simulateTimestep(float timeStep)
{
	// calculation for each timestep
	Mat4 mm;
	Mat4 AM, BM;
	Mat4 AM_sca;
	Mat4 AM_rot;
	Mat4 AM_tra;
	Mat4 BM_sca;
	Mat4 BM_rot;
	Mat4 BM_tra;
	CollisionInfo collision_info;
	float jjj;
	float elas = 1;
	Vec3 xa;
	Vec3 xb;
	Vec3 va;
	Vec3 vb;
	Vec3 wa;
	Vec3 wb;
	int ma;
	int mb;
	Vec3 n;
	Mat4 Iai;
	Mat4 Ibi;


	for (int idx = 0; idx < rigid_body_list.size(); idx++)
	{
		
		rigid_body_list[idx].orientation =  1 / rigid_body_list[idx].orientation.norm() * rigid_body_list[idx].orientation;

		for (int idy = 0; idy < rigid_body_list.size() - 1; idy++)
		{
			int idd = idy + idx + 1;
			if (idd >= rigid_body_list.size())
				idd = idd - rigid_body_list.size();

			AM_sca = Mat4(
				rigid_body_list[idx].size.x, 0, 0, 0,
				0, rigid_body_list[idx].size.y, 0, 0,
				0, 0, rigid_body_list[idx].size.z, 0,
				0, 0, 0, 1
			);

			AM_rot = rigid_body_list[idx].orientation.getRotMat().inverse();

			AM_tra = Mat4(
				1, 0, 0, 0,
				0, 1, 0, 0,
				0, 0, 1, 0,
				rigid_body_list[idx].position_mass_center.x, rigid_body_list[idx].position_mass_center.y, rigid_body_list[idx].position_mass_center.z, 1
			);
			
			AM = AM_sca * AM_rot * AM_tra;
			
			BM_sca = Mat4(
				rigid_body_list[idd].size.x, 0, 0, 0,
				0, rigid_body_list[idd].size.y, 0, 0,
				0, 0, rigid_body_list[idd].size.z, 0,
				0, 0, 0, 1
			);

			BM_rot = rigid_body_list[idd].orientation.getRotMat().inverse();

			BM_tra.initTranslation(rigid_body_list[idd].position_mass_center.x, rigid_body_list[idd].position_mass_center.y, rigid_body_list[idd].position_mass_center.z);
			
			BM = BM_sca * BM_rot * BM_tra;

			collision_info = checkCollisionSAT(AM, BM);

			xa = collision_info.collisionPointWorld - rigid_body_list[idx].position_mass_center;
			xb = collision_info.collisionPointWorld - rigid_body_list[idd].position_mass_center;
			va = rigid_body_list[idx].linear_velocity_mass_center;
			vb = rigid_body_list[idd].linear_velocity_mass_center;
			wa = rigid_body_list[idx].angular_velocity_mass_center;
			wb = rigid_body_list[idd].angular_velocity_mass_center;
			ma = rigid_body_list[idx].whole_mass;
			mb = rigid_body_list[idd].whole_mass;
			n = collision_info.normalWorld;

			if (collision_info.isValid)
			{
				jjj = (-1 - elas) * dot((va + cross(xa, wa)) - (vb + cross(xb, wb)), n);

				Iai = Mat4(
					1 / rigid_body_list[idx].inertia_tensor.x, 0, 0, 0,
					0, 1 / rigid_body_list[idx].inertia_tensor.y, 0, 0,
					0, 0, 1 / rigid_body_list[idx].inertia_tensor.z, 0,
					0, 0, 0, 1
				);

				Iai = rigid_body_list[idx].orientation.getRotMat() * Iai * rigid_body_list[idx].orientation.getRotMat().inverse();

				Ibi = Mat4(
					1 / rigid_body_list[idd].inertia_tensor.x, 0, 0, 0,
					0, 1 / rigid_body_list[idd].inertia_tensor.y, 0, 0,
					0, 0, 1 / rigid_body_list[idd].inertia_tensor.z, 0,
					0, 0, 0, 1
				);

				Ibi = rigid_body_list[idd].orientation.getRotMat() * Ibi * rigid_body_list[idd].orientation.getRotMat().inverse();

				jjj = jjj / (1 / ma + 1 / mb + dot(cross((Iai * cross(xa, n)), xa) + cross((Ibi * cross(xb, n)), xb), n));


				rigid_body_list[idx].linear_velocity_mass_center = rigid_body_list[idx].linear_velocity_mass_center + jjj * n / ma;
				rigid_body_list[idx].angular_momentum = rigid_body_list[idx].angular_momentum + cross(xa, jjj * n);
			}
			

		}
		
		rigid_body_list[idx].position_mass_center = rigid_body_list[idx].position_mass_center + timeStep * rigid_body_list[idx].linear_velocity_mass_center;
		
		rigid_body_list[idx].linear_velocity_mass_center = rigid_body_list[idx].linear_velocity_mass_center + timeStep / rigid_body_list[idx].whole_mass * rigid_body_list[idx].force;

		rigid_body_list[idx].orientation = rigid_body_list[idx].orientation + timeStep / 2 * Quat(rigid_body_list[idx].angular_velocity_mass_center.x, rigid_body_list[idx].angular_velocity_mass_center.y, rigid_body_list[idx].angular_velocity_mass_center.z, 0) * rigid_body_list[idx].orientation;
		rigid_body_list[idx].angular_momentum = rigid_body_list[idx].angular_momentum + timeStep * rigid_body_list[idx].torque;

		mm = Mat4(
			1/rigid_body_list[idx].inertia_tensor.x,0,0,0,
			0, 1/rigid_body_list[idx].inertia_tensor.y,0,0,
			0,0, 1/rigid_body_list[idx].inertia_tensor.z,0,
			0,0,0,1
		);
		rigid_body_list[idx].angular_velocity_mass_center = rigid_body_list[idx].orientation.getRotMat() * mm * rigid_body_list[idx].orientation.getRotMat().inverse() * rigid_body_list[idx].angular_momentum; //WTF?

		for (int idy = 0; idy < rigid_body_list[idx].mass_point_list.size(); idy++)
		{
			rigid_body_list[idx].mass_point_list[idy].position = rigid_body_list[idx].position_mass_center + rigid_body_list[idx].orientation.getRotMat() * rigid_body_list[idx].mass_point_list[idy].relative_position;
			rigid_body_list[idx].mass_point_list[idy].velocity = rigid_body_list[idx].linear_velocity_mass_center + cross(rigid_body_list[idx].angular_velocity_mass_center, rigid_body_list[idx].mass_point_list[idy].relative_position);
		}

		rigid_body_list[idx].force = Vec3(0, 0, 0);
		rigid_body_list[idx].torque = Vec3(0, 0, 0);
	}
}

void RigidBodySystemSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void RigidBodySystemSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

int RigidBodySystemSimulator::getNumberOfRigidBodies()
{
	int outcome = 0;

	outcome = rigid_body_list.size();

	return outcome;
}
Vec3 RigidBodySystemSimulator::getPositionOfRigidBody(int i)
{
	Vec3 outcome = Vec3(0, 0, 0);

	if (i < rigid_body_list.size())
	{
		outcome = rigid_body_list[i].position_mass_center;
	}

	return outcome;
}

Vec3 RigidBodySystemSimulator::getLinearVelocityOfRigidBody(int i)
{
	Vec3 outcome = Vec3(0, 0, 0);

	if (i < rigid_body_list.size())
	{
		outcome = rigid_body_list[i].linear_velocity_mass_center;
	}

	return outcome;
}

Vec3 RigidBodySystemSimulator::getAngularVelocityOfRigidBody(int i)
{
	Vec3 outcome = Vec3(0, 0, 0);

	if (i < rigid_body_list.size())
	{
		outcome.x = rigid_body_list[i].angular_momentum.x / rigid_body_list[i].inertia_tensor.x;
		outcome.y = rigid_body_list[i].angular_momentum.y / rigid_body_list[i].inertia_tensor.y;
		outcome.z = rigid_body_list[i].angular_momentum.z / rigid_body_list[i].inertia_tensor.z;
	}
	
	return outcome;
}

void RigidBodySystemSimulator::applyForceOnBody(int i, Vec3 loc, Vec3 force)
{
	if (i < rigid_body_list.size())
	{
		rigid_body_list[i].force = rigid_body_list[i].force + force;
		rigid_body_list[i].torque = rigid_body_list[i].torque + cross(loc, force);
	}
	else
	{
		std::cout << "Out of rigid body list!" << std::endl;
	}
}

void RigidBodySystemSimulator::addRigidBody(Vec3 position, Vec3 size, int mass)
{
	RigidBody rb;

	rb.position_mass_center = position;
	rb.size = size;
	rb.whole_mass = mass;
	rb.linear_velocity_mass_center = Vec3(0, 0, 0);
	rb.angular_velocity_mass_center = Vec3(0, 0, 0);
	rb.orientation = Quat(0, 0, 1, 0);//?
	rb.angular_momentum = Vec3(0, 0, 0);

	//XMMatrixRotationQuaternion

	for (int i = 0; i < 2; i++)
	{
		for (int j = 0; j < 2; j++)
		{
			for (int k = 0; k < 2; k++)
			{
				MessPoint mp;
				mp.velocity = Vec3(0, 0, 0);
				mp.relative_position = Vec3(i * 2 - 1, j * 2 - 1, k * 2 - 1) * size * 0.5;
				mp.position = mp.relative_position + position;
				rb.mass_point_list.push_back(mp);
			}
		}
	}

	rb.inertia_tensor = Vec3(mass * (size.z * size.z + size.x * size.x) / 12, mass * (size.y * size.y + size.z * size.z) / 12, mass * (size.y * size.y + size.x * size.x) / 12);

	rigid_body_list.push_back(rb);
}

void RigidBodySystemSimulator::setOrientationOf(int i, Quat orientation)
{
	if (i < rigid_body_list.size())
	{
		rigid_body_list[i].orientation = orientation;
	}
	else
	{
		std::cout << "Out of rigid body list!" << std::endl;
	}
}

void RigidBodySystemSimulator::setVelocityOf(int i, Vec3 velocity)
{
	if (i < rigid_body_list.size())
	{
		rigid_body_list[i].linear_velocity_mass_center = velocity;
		for (int idx = 0; idx < 8; idx++)
		{
			rigid_body_list[i].mass_point_list[i].velocity = velocity;
		}
	}
	else
	{
		std::cout << "Out of rigid body list!" << std::endl;
	}
}

void RigidBodySystemSimulator::setAngularVelocityOf(int i)
{
	if (i < rigid_body_list.size())
	{
		/*rigid_body_list[i].angular_velocity_mass_center = velocity;*/

	}
	else
	{
		std::cout << "Out of rigid body list!" << std::endl;
	}
}