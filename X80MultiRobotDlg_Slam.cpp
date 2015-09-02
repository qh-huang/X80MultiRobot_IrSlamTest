#include "stdafx.h"
#include "X80MultiRobot.h"
#include "X80MultiRobotDlg.h"
#include <cmath>
#include <cstring>
#include <vector>
#include "include/gmapping/utils/GPoint.h"
#include "include/Infrared.h"
#include <conio.h>
#define _USE_MATH_DEFINES

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

#define NO_CONTROL -32768
#define M_PWM 0
#define M_POSITION 1
#define M_VELOCITY 2
#define cFULL_COUNT 32767
#define cWHOLE_RANGE 1200

#define NELEMS(x)  (sizeof(x) / sizeof((x)[0]))

const UINT ID_TIMER_UPDATE = 0x1001;
const UINT ID_TIMER_PROCESS_SCAN = 0x1002;
const UINT UPDATE_ELAPSE = 50;
const double CONST_PI = 3.141592653589793238463;
const uint16_t ROBOT_MAP_SIZE = 500;
#define NUM_IR 6
//#define NUM_IR 1
Infrared ir_readings[NUM_IR];

// qiao@2015.09.01: add CSV utils
std::vector<std::string> getNextLineAndSplitIntoTokens(std::istream& str)
{
	std::vector<std::string>   result;
	std::string                line;
	std::getline(str, line);

	std::stringstream          lineStream(line);
	std::string                cell;

	while (std::getline(lineStream, cell, ','))
	{
		result.push_back(cell);
	}
	return result;
}
class CSVRow
{
public:
	std::string const& operator[](std::size_t index) const
	{
		return m_data[index];
	}
	std::size_t size() const
	{
		return m_data.size();
	}
	void readNextRow(std::istream& str)
	{
		std::string         line;
		std::getline(str, line);

		std::stringstream   lineStream(line);
		std::string         cell;

		m_data.clear();
		while (std::getline(lineStream, cell, ','))
		{
			m_data.push_back(cell);
		}
	}
private:
	std::vector<std::string>    m_data;
};
std::istream& operator>>(std::istream& str, CSVRow& data)
{
	data.readNextRow(str);
	return str;
}
class CSVIterator
{
public:
	typedef std::input_iterator_tag     iterator_category;
	typedef CSVRow                      value_type;
	typedef std::size_t                 difference_type;
	typedef CSVRow*                     pointer;
	typedef CSVRow&                     reference;

	CSVIterator(std::istream& str) :m_str(str.good() ? &str : NULL) { ++(*this); }
	CSVIterator() :m_str(NULL) {}

	// Pre Increment
	CSVIterator& operator++() 
	{ 
		if (m_str) { (*m_str) >> m_row; m_str = m_str->good() ? m_str : NULL; }
		return *this; 
	}
	// Post increment
	CSVIterator operator++(int) { CSVIterator    tmp(*this); ++(*this); return tmp; }
	CSVRow const& operator*()   const { return m_row; }
	CSVRow const* operator->()  const { return &m_row; }

	bool operator==(CSVIterator const& rhs) { return ((this == &rhs) || ((this->m_str == NULL) && (rhs.m_str == NULL))); }
	bool operator!=(CSVIterator const& rhs) { return !((*this) == rhs); }
private:
	std::istream*       m_str;
	CSVRow              m_row;
};
// end qiao@2015.09.01: add CSV utils

/* qiao@2015.08.11:
// IR data too unstable, use ring-buffer to smooth
template<typename T>
struct SimpleRingBuffer
{
static const uint16_t BUFFER_SIZE = 32;
std::vector<T> _vBuf;
uint16_t idx;
T sum;
SimpleRingBuffer() { idx = 0; sum = 0; }
void add(T t)
{
if (_vBuf.size() < BUFFER_SIZE) {
_vBuf.push_back(t);
sum += t;
}
else {
sum -= _vBuf[idx];
_vBuf[idx] = t;
sum += t;
idx++;
idx %= BUFFER_SIZE;
}
}
double getAverage() { return (double)sum / BUFFER_SIZE;  }
};

SimpleRingBuffer<double> IR1_Array;
SimpleRingBuffer<double> IR2_Array;
SimpleRingBuffer<double> IR3_Array;
SimpleRingBuffer<double> IR4_Array;
SimpleRingBuffer<double> IR5_Array;
SimpleRingBuffer<double> IR6_Array;
SimpleRingBuffer<double> IR7_Array;
*/
UINT AD2Dis(short IRValue) // qiao@2015.08.10: copy from csharp project
{
	double temp = 0;
	double IRAD2Distance = 0;

	temp = 21.6 / ((double)IRValue * 3 / 4028 - 0.17);

	// IR range 10-80cm
	if ((temp > 80) || (temp < 0))
		IRAD2Distance = 81;
	else if ((temp < 10) && (temp > 0))
		IRAD2Distance = 9;
	else
		IRAD2Distance = temp;
	return IRAD2Distance;
}

// qiao
UINT MyThreadProc(LPVOID pParam)
{
	CX80MultiRobotDlg* pCX80Dlg = (CX80MultiRobotDlg*)pParam;

	if (pCX80Dlg == NULL) return 1;   // if pObject is not valid

									  // do something with 'pCX80Dlg'
	pCX80Dlg->PostMessageA(WM_COMMAND, MAKELONG(IDC_TurnLeft, BN_CLICKED), (LPARAM)pCX80Dlg->GetDlgItem(IDC_TurnLeft)->GetSafeHwnd());
	// MessageBox(*pCX80Dlg, "MyThreadProc2", 0, 0);

	return 0;   // thread completed successfully
}
// qiao(end)

CX80MultiRobotDlg::~CX80MultiRobotDlg() 
{ 
	cv::destroyAllWindows();
	m_oFile.close();
}

void CX80MultiRobotDlg::RobotParamInit()
{
	const double ROBOT_SPEED_DEFAULT = 300.00;
	m_VelocityL = ROBOT_SPEED_DEFAULT;
	m_VelocityR = ROBOT_SPEED_DEFAULT - 4;

	m_Mat = cv::Mat(ROBOT_MAP_SIZE, ROBOT_MAP_SIZE, CV_8UC3, cv::Scalar(0));
//	m_Pose = mrpt::poses::CPose2D(0.0, 0.0, PI / 2);
	m_Pose = GMapping::OrientedPoint(0.0, 0.0, 0.0);
	m_Moving = false;

	//m_got_first_scan_ = false;
	const double maxUrange_ = 0.6;// 60.0;
	const double maxRange_ = 0.8;// 80.0;
	const double sigma_ = 0.05;
	const int kernelSize_ = 3;
	const double lstep_ = 0.15;// find best pose within about +-15 cm
	const double astep_ = CONST_PI / 18.0; // find best pose within about +-10 degree
	const int iterations_ = 8.0;
	const double lsigma_ = 0.0;
	const unsigned int lskip_ = 0.0;
	m_matcher.setMatchingParameters(maxUrange_, maxRange_, sigma_, kernelSize_, lstep_, astep_, iterations_, lsigma_, lskip_);
	m_matcher.setgenerateMap(true);

	m_count = 0;

	delta_ = 0.05;
	occ_thresh_ = 0.5;

	GMapping::Point center;
	double xmin_ = -5.0;
	double ymin_ = -5.0;
	double xmax_ = 5.0;
	double ymax_ = 5.0;
	center.x = (xmin_ + xmax_) / 2.0;
	center.y = (ymin_ + ymax_) / 2.0;
	m_smmap = GMapping::ScanMatcherMap(center, xmin_, ymin_, xmax_, ymax_, delta_);

	m_flagStart = false;
	m_minimumScore = 5.5;
}

// RobotSensorUpdate: get IR data, Sonar, Odometry data
void CX80MultiRobotDlg::RobotSensorUpdate()
{
	// calculate distance
	//m_IR_1 = AD2Dis(m_IR1);
	//m_IR_2 = AD2Dis(m_IR2);
	//m_IR_3 = AD2Dis(m_IR3);
	//m_IR_4 = AD2Dis(m_IR4);
	//m_IR_5 = AD2Dis(m_IR5);
	//m_IR_6 = AD2Dis(m_IR6);
	//m_IR_7 = AD2Dis(m_IR7);
	m_Sonar_1 = m_Sonar1;
	m_Sonar_2 = m_Sonar2;
	m_Sonar_3 = m_Sonar3;

	m_IR_1 = AD2Dis(m_IR1);
	m_IR_2 = AD2Dis(m_IR2);
	m_IR_3 = AD2Dis(m_IR3);
	m_IR_4 = AD2Dis(m_IR4);
	m_IR_5 = AD2Dis(m_IR5);
	m_IR_6 = AD2Dis(m_IR6);
	m_IR_7 = AD2Dis(m_IR7);
	/*
	m_IR_1 = AD2Dis(IR1_Array.getAverage());
	m_IR_2 = AD2Dis(IR2_Array.getAverage());
	m_IR_3 = AD2Dis(IR3_Array.getAverage());
	m_IR_4 = AD2Dis(IR4_Array.getAverage());
	m_IR_5 = AD2Dis(IR5_Array.getAverage());
	m_IR_6 = AD2Dis(IR6_Array.getAverage());
	m_IR_7 = AD2Dis(IR7_Array.getAverage());
	*/
	UpdateData(false);

	/*
	// according to IR position, IR right, right-front, right-head, left-head, left-front, left
	// is IR 5,4,3,2,1,7
	const double ANGLE_5 = 0.0;	// rad, about 180-51.44 deg
	const double ANGLE_4 = 0.897857996 + 0.2;	// rad, about 51.44 deg
	const double ANGLE_3 = 1.31347261;	// rad, about 75.2564384 deg
	const double ANGLE_2 = PI - ANGLE_3;
	const double ANGLE_1 = PI - ANGLE_4;
	const double ANGLE_7 = PI;
	const double ANGLE_6 = -PI / 2.0;
	const double OFFSET_1 = 22.0;
	const double OFFSET_2 = 22.0;
	const double OFFSET_3 = 22.0;
	const double OFFSET_4 = 22.0;
	const double OFFSET_5 = 10.0;
	const double OFFSET_6 = 22.0;
	const double OFFSET_7 = 10.0;

	
	//mrpt::poses::CPoint2D IR_Point[7];
	IR_Point[0].x((m_IR_1 - OFFSET_1)*std::cos(ANGLE_1));
	IR_Point[0].y((m_IR_1 - OFFSET_1)*std::sin(ANGLE_1));
	IR_Point[1].x((m_IR_2 - OFFSET_2)*std::cos(ANGLE_2));
	IR_Point[1].y((m_IR_2 - OFFSET_2)*std::sin(ANGLE_2));
	IR_Point[2].x((m_IR_3 - OFFSET_3)*std::cos(ANGLE_3));
	IR_Point[2].y((m_IR_3 - OFFSET_3)*std::sin(ANGLE_3));
	IR_Point[3].x((m_IR_4 - OFFSET_4)*std::cos(ANGLE_4));
	IR_Point[3].y((m_IR_4 - OFFSET_4)*std::sin(ANGLE_4));
	IR_Point[4].x((m_IR_5 - OFFSET_5)*std::cos(ANGLE_5));
	IR_Point[4].y((m_IR_5 - OFFSET_5)*std::sin(ANGLE_5));
	IR_Point[5].x((m_IR_6 - OFFSET_6)*std::cos(ANGLE_6));
	IR_Point[5].y((m_IR_6 - OFFSET_6)*std::sin(ANGLE_6));
	IR_Point[6].x((m_IR_7 - OFFSET_7)*std::cos(ANGLE_7));
	IR_Point[6].y((m_IR_7 - OFFSET_7)*std::sin(ANGLE_7));
	//
	GMapping::Point IR_Point[7];
	IR_Point[0].x = (m_IR_1 - OFFSET_1)*std::cos(ANGLE_1);
	IR_Point[0].y = (m_IR_1 - OFFSET_1)*std::sin(ANGLE_1);
	IR_Point[1].x = (m_IR_2 - OFFSET_2)*std::cos(ANGLE_2);
	IR_Point[1].y = (m_IR_2 - OFFSET_2)*std::sin(ANGLE_2);
	IR_Point[2].x = (m_IR_3 - OFFSET_3)*std::cos(ANGLE_3);
	IR_Point[2].y = (m_IR_3 - OFFSET_3)*std::sin(ANGLE_3);
	IR_Point[3].x = (m_IR_4 - OFFSET_4)*std::cos(ANGLE_4);
	IR_Point[3].y = (m_IR_4 - OFFSET_4)*std::sin(ANGLE_4);
	IR_Point[4].x = (m_IR_5 - OFFSET_5)*std::cos(ANGLE_5);
	IR_Point[4].y = (m_IR_5 - OFFSET_5)*std::sin(ANGLE_5);
	IR_Point[5].x = (m_IR_6 - OFFSET_6)*std::cos(ANGLE_6);
	IR_Point[5].y = (m_IR_6 - OFFSET_6)*std::sin(ANGLE_6);
	IR_Point[6].x = (m_IR_7 - OFFSET_7)*std::cos(ANGLE_7);
	IR_Point[6].y = (m_IR_7 - OFFSET_7)*std::sin(ANGLE_7);

	// clear image
	m_Mat.setTo(cv::Scalar(0.0, 0.0, 0.0));
	// draw robot
	const int THICKNESS_FILLED = -1; // filled
	cv::circle(m_Mat, cv::Point(m_Pose.x + (ROBOT_MAP_SIZE / 2), (ROBOT_MAP_SIZE / 2) - m_Pose.y), 10, cv::Scalar(255, 0, 0), THICKNESS_FILLED, CV_AA);
	// draw IR
	const double SCALE = 3.0;
	// debug
	m_Log.Empty();
	for (uint16_t i = 0; i < 7; i++) {
		cv::line(
			m_Mat,
			cv::Point(m_Pose.x + (ROBOT_MAP_SIZE / 2), (ROBOT_MAP_SIZE / 2) - m_Pose.y),
			cv::Point(SCALE*IR_Point[i].x + (ROBOT_MAP_SIZE / 2), (ROBOT_MAP_SIZE / 2) - SCALE*IR_Point[i].y),
			cv::Scalar(0, 255, 0),
			2,
			CV_AA);
		char tmp[33];
		std::string str("");
		str += "P[";
		str += itoa(i, tmp, 10);
		str += "] = (";
		str += itoa(IR_Point[i].x, tmp, 10);
		str += ",";
		str += itoa(IR_Point[i].y, tmp, 10);
		str += ")\n";
		m_Log.Append(str.c_str());
	}
	cv::line(
		m_Mat,
		cv::Point(SCALE*IR_Point[0].x + (ROBOT_MAP_SIZE / 2), (ROBOT_MAP_SIZE / 2) - SCALE*IR_Point[0].y),
		cv::Point(SCALE*IR_Point[6].x + (ROBOT_MAP_SIZE / 2), (ROBOT_MAP_SIZE / 2) - SCALE*IR_Point[6].y),
		cv::Scalar(255, 255, 0),
		2,
		CV_AA);
	UpdateData(false);

	cv::imshow("Robot", m_Mat);
	*/
}

BOOL CX80MultiRobotDlg::PreTranslateMessage(MSG * pMsg)
{
	// up, down, left, right
	// --> w,s,a,d (keycode: 87 83 65 68)
	// --> arraw keys (keycode: 38 40 37 39)
	// --> num key (104 98 100 102) (56 50 52 54)

	//CString a;
	switch (pMsg->message) {
	case WM_KEYDOWN:
		if (!m_Moving) {
			switch (pMsg->wParam) {
				// Forward
			case 87:
			case 38:
			case 56:
			case 104:
				// forward
				m_MOTSDK.SetDcMotorControlMode(0, M_VELOCITY);
				m_MOTSDK.SetDcMotorControlMode(1, M_VELOCITY);
				m_MOTSDK.SetDcMotorVelocityControlPID(0, 10, 10, 0); //TODO: modify PID parameters
				m_MOTSDK.SetDcMotorVelocityControlPID(1, 10, 10, 0);
				m_MOTSDK.DcMotorVelocityNonTimeCtrAll(-m_VelocityL, m_VelocityR, NO_CONTROL, NO_CONTROL, NO_CONTROL, NO_CONTROL);
				m_Moving = true;
				//MessageBox("forward");
				break;
			case 65:
			case 37:
			case 100:
			case 52:
				// TODO: left
				m_MOTSDK.SetDcMotorControlMode(0, M_VELOCITY);
				m_MOTSDK.SetDcMotorControlMode(1, M_VELOCITY);
				m_MOTSDK.SetDcMotorVelocityControlPID(0, 30, 20, 0);
				m_MOTSDK.SetDcMotorVelocityControlPID(1, 30, 20, 0);
				m_MOTSDK.DcMotorVelocityNonTimeCtrAll(m_VelocityL / 1.5, m_VelocityR / 1.5, NO_CONTROL, NO_CONTROL, NO_CONTROL, NO_CONTROL);
				//MessageBox("left turn");
				m_Moving = true;
				break;
			case 68:
			case 39:
			case 102:
			case 54:
				// TODO: right
				m_MOTSDK.SetDcMotorControlMode(0, M_VELOCITY);
				m_MOTSDK.SetDcMotorControlMode(1, M_VELOCITY);
				m_MOTSDK.SetDcMotorVelocityControlPID(0, 30, 10, 0);
				m_MOTSDK.SetDcMotorVelocityControlPID(1, 30, 10, 0);
				m_MOTSDK.DcMotorVelocityNonTimeCtrAll(-m_VelocityL / 1.5, -m_VelocityR / 1.5, NO_CONTROL, NO_CONTROL, NO_CONTROL, NO_CONTROL);
				//MessageBox("right turn");
				m_Moving = true;
				break;
			default:
				;
			}
		}
		//if (pMsg->wParam == 66)
		//a.Format(_T("%d"), pMsg->wParam);
		//MessageBox(a);
		break;
	case WM_KEYUP: // stop all behavior
				   //if (pMsg->wParam == 65)
				   //MessageBox("key up");
		m_MOTSDK.DisableDcMotor(0);
		m_MOTSDK.DisableDcMotor(1);
		m_Moving = false;
		break;
	default:
		;
	}
	/*
	if (pMsg->message == WM_KEYDOWN) {
	if (pMsg->wParam == 65)
	MessageBox("key down");
	}
	*/
	return CDialog::PreTranslateMessage(pMsg);
}

void CX80MultiRobotDlg::OnTimer(UINT nIDEvent)
{
	// TODO: Add your message handler code here and/or call default
	//m_MOTSDK.TakePhoto();

	// qiao@2015.08.10
	switch (nIDEvent) {
	case ID_TIMER_UPDATE:
		RobotSensorUpdate();	// update IR, Odometry
		RobotPoseUpdate_Odometry();
		RobotPoseUpdate_ProcessScan();
		RobotDebugCV();
		break;
	case ID_TIMER_PROCESS_SCAN:
//		RobotPoseUpdate_ProcessScan();
		//MessageBeep(500);
		break;
	default:
		;
	} // end switch (nIDEvent)
	CDialog::OnTimer(nIDEvent);
}

void CX80MultiRobotDlg::RobotPoseUpdate_ProcessScan() 
{
	if (!m_flagStart) { return; }

	// according to IR position, IR right, right-front, right-head, left-head, left-front, left
	// is IR 5,4,3,2,1,7
	const double ANGLE_5 = 0.0;	// rad, about 180-51.44 deg
	const double ANGLE_4 = 0.897857996 + 0.2;	// rad, about 51.44 deg
	const double ANGLE_3 = 1.31347261;	// rad, about 75.2564384 deg
	const double ANGLE_2 = CONST_PI - ANGLE_3;
	const double ANGLE_1 = CONST_PI - ANGLE_4;
	const double ANGLE_7 = CONST_PI;
	//const double ANGLE_6 = -CONST_PI / 2.0;
	const unsigned int BEANS = 4;
	const double RES = 0.05; // angle resolution
	const double MAXRANGE = 60;

	const double OFFSET_1 = 0.18;
	const double OFFSET_2 = 0.18;
	const double OFFSET_3 = 0.18;
	const double OFFSET_4 = 0.18;
	const double OFFSET_5 = 0.10;
	//const double OFFSET_6 = 0.18;
	const double OFFSET_7 = 0.10;


	ir_readings[0] = Infrared(OFFSET_1+(double)m_IR_1 / 100.0, GMapping::OrientedPoint(0, 0, ANGLE_1), MAXRANGE);
	ir_readings[1] = Infrared(OFFSET_2 + (double)m_IR_2 / 100.0, GMapping::OrientedPoint(0, 0, ANGLE_2), MAXRANGE);
	ir_readings[2] = Infrared(OFFSET_3 + (double)m_IR_3 / 100.0, GMapping::OrientedPoint(0, 0, ANGLE_3), MAXRANGE);
	ir_readings[3] = Infrared(OFFSET_4 + (double)m_IR_4 / 100.0, GMapping::OrientedPoint(0, 0, ANGLE_4), MAXRANGE);
	ir_readings[4] = Infrared(OFFSET_5 + (double)m_IR_5 / 100.0, GMapping::OrientedPoint(0, 0, ANGLE_5), MAXRANGE);
	//ir_readings[5] = Infrared(OFFSET_6+(double)m_IR_6 / 100.0, GMapping::OrientedPoint(0, 0, ANGLE_6), MAXRANGE);
	ir_readings[5] = Infrared(OFFSET_7 + (double)m_IR_7 / 100.0, GMapping::OrientedPoint(0, 0, ANGLE_7), MAXRANGE);
	//ir_readings[6] = Infrared(OFFSET_7+(double)m_IR_7 / 100.0, GMapping::OrientedPoint(0, 0, ANGLE_7), MAXRANGE);

	// correct / update pose
	if (m_count > 0) {
		// refer to GridSlamProcessor::scanMatch
		double sumScore = 0;
		GMapping::OrientedPoint corrected_pose;
		double score;// , l, s;
		score = m_matcher.optimize(corrected_pose, m_smmap, m_Pose, ir_readings);
		m_dbgPose = corrected_pose;
		_cprintf("ProcessScan:\n");
		//system("pause");
		if (score > m_minimumScore) {
			_cprintf("score=%lf > minimumScore %lf, Estimated Pose(x,y,theta)=(%lf,%lf,%lf)\n\n",
				score, m_minimumScore, corrected_pose.x, corrected_pose.y, corrected_pose.theta * 180 / CONST_PI);
			//m_Pose = corrected_pose;
			//system("pause");
		} else {
		// show score for debug 
			_cprintf("score=%lf <= minimumScore %lf, Estimated Pose(x,y,theta)=(%lf,%lf,%lf)\n\n",
				score, m_minimumScore, corrected_pose.x, corrected_pose.y, corrected_pose.theta * 180 / CONST_PI);
		}
	}

	// register scan
	m_matcher.invalidateActiveArea();
	m_matcher.computeActiveArea(m_smmap, m_Pose, ir_readings);
	m_matcher.registerScan(m_smmap, m_Pose, ir_readings);
	m_count++;
}

void CX80MultiRobotDlg::RobotPoseUpdate_Odometry() 
{
	double delta_Encoder1 = m_Encoder1 - m_Encoder1_prev;
	double delta_Encoder2 = m_Encoder2 - m_Encoder2_prev;
	if (fabs(delta_Encoder1) < 10 && fabs(delta_Encoder2) < 10) { return; }
	
	/* debug
	m_Log.Empty();
	std::string str("");
	
	str += "dEnc1=";
	str += std::to_string(delta_Encoder1);
	str += ", dEnc2=";
	str += std::to_string(delta_Encoder1); 
	str += "\n\n";
	*/
	//_cprintf("dEnc1=%lf, dEnc2=%lf\n", delta_Encoder1, delta_Encoder2);
	// check if encoder just over the maximum (default 32767)
	const double THRESHOLD = (double)cFULL_COUNT * 0.8;
	if (delta_Encoder1 > 0 && delta_Encoder1 > THRESHOLD) { // decreasing and just cross 0->cFULL_COUNT
		delta_Encoder1 = delta_Encoder1 - (double)cFULL_COUNT;
	}
	if (delta_Encoder1 < 0 && delta_Encoder1 < -THRESHOLD) { // increasing and just cross cFULL_COUNT->0
		delta_Encoder1 = delta_Encoder1 + (double)cFULL_COUNT;
	}
	if (delta_Encoder2 > 0 && delta_Encoder2 > THRESHOLD) { // decreasing and just cross 0->cFULL_COUNT
		delta_Encoder2 = delta_Encoder2 - (double)cFULL_COUNT;
	}
	if (delta_Encoder2 < 0 && delta_Encoder2 < -THRESHOLD) { // increasing and just cross cFULL_COUNT->0
		delta_Encoder2 = delta_Encoder2 + (double)cFULL_COUNT;
	}

	/*
	_cprintf("\n");
	_cprintf("Enc1=%f, Enc2=%f\n", m_Encoder1, m_Encoder2);
	_cprintf("dEnc1=%lf, dEnc2=%lf\n", delta_Encoder1, delta_Encoder2);
	*/
	// turn right: both delta_Encoder < 0
	// turn left: both delta_Encoder > 0
	// forward: delta_Encoder1 < 0, delta_Encoder2 > 0
	
	// for rotation, just calculate delta angle and apply
	const double FULL_COUNT_1 = 2480;
	const double FULL_COUNT_2 = 2480;
	if (delta_Encoder1 * delta_Encoder2 >= 0) {
		double dAngleD_by_Enc1 = 360.0 * delta_Encoder1 / FULL_COUNT_1;
		double dAngleD_by_Enc2 = 360.0 * delta_Encoder2 / FULL_COUNT_2;
		double dAngle_by_Enc1 = dAngleD_by_Enc1 * CONST_PI / 180.0;
		double dAngle_by_Enc2 = dAngleD_by_Enc2 * CONST_PI / 180.0;
		/*
		_cprintf("\n");
		_cprintf("dAngle_by_Enc1=%lf, dAngle_by_Enc2=%lf\n", 
			dAngle_by_Enc1, dAngle_by_Enc2);
		*/
		//m_Pose.rotate(dAngle_by_Enc1 + dAngle_by_Enc2);
		m_Pose.theta += (dAngle_by_Enc1 + dAngle_by_Enc2);
		if (m_Pose.theta > 2 * CONST_PI) {
			m_Pose.theta -= 2 * CONST_PI;
		}
		if (m_Pose.theta < -2 * CONST_PI) {
			m_Pose.theta += 2 * CONST_PI;
		}
	} else {
	// for transition, need to find distance and delta angle
		//const double CM_PER_COUNT = 1.2 / 3360;
		const double CM_PER_COUNT = 1.5 / 3360;
		// forward: delta+_Encoder1 < 0, delta_Encoder2 > 0
		double dAngle = 2 * CONST_PI * (delta_Encoder1 + delta_Encoder2) / ((FULL_COUNT_1 + FULL_COUNT_2) / 2.0);
		double deltaEncoder = (fabs(delta_Encoder1) < fabs(delta_Encoder2)) ? delta_Encoder1 : delta_Encoder2;
		double direction = (delta_Encoder2 > 0) ? 1.0 : -1.0;
		double dDistance = direction * fabs(deltaEncoder) * CM_PER_COUNT;
		m_Pose.theta += (dAngle / 2.0); //m_Pose.rotate(dAngle);
		m_Pose.x += dDistance * cos(m_Pose.theta + CONST_PI / 2.0);
		m_Pose.y += dDistance * sin(m_Pose.theta + CONST_PI / 2.0);
		m_Pose.theta += (dAngle / 2.0); //m_Pose.rotate(dAngle);
	}
	/*
	_cprintf("\n");
	_cprintf("m_Pose.(x,y,theta)=(%lf,%lf,%lf)\n", m_Pose.x, m_Pose.y, m_Pose.theta * 180 / CONST_PI);
	_cprintf("\n\n");
	*/

	// draw it, debug only
	// clear image
	// update
	m_Encoder1_prev = m_Encoder1;
	m_Encoder2_prev = m_Encoder2;
}

void CX80MultiRobotDlg::RobotDebugCV() 
{
	// (re)allocate matrix memory
	/*
	_cprintf("smmap.getMapSizeX=%d, smmap.getMapSizeY=%d\n", m_smmap.getMapSizeX(), m_smmap.getMapSizeY());
	_cprintf("smmap.center=(%lf,%lf)\n", m_smmap.getCenter().x, m_smmap.getCenter().y);
	_cprintf("smmap.delta=%lf\n", m_smmap.getDelta());
	_cprintf("smmap.MapResolution=%lf\n", m_smmap.getMapResolution());
	_cprintf("smmap.Resolution=%lf\n", m_smmap.getResolution());
	_cprintf("smmap.WorldSize=(%lf,%lf)\n", m_smmap.getWorldSizeX(), m_smmap.getWorldSizeY());
	*/
	const int GRID_SIZE = 2;
	const int LINE_WIDTH = 2;
	m_Mat.create(m_smmap.getMapSizeX() * (GRID_SIZE+ LINE_WIDTH), m_smmap.getMapSizeY() * (GRID_SIZE+LINE_WIDTH), CV_8UC3);
	m_Mat.setTo(cv::Scalar(.0, .0, .0));
	// refer to SlamGMapping::updateMap
	//GMapping::Point center((double)m_smmap.getMapSizeX()/2.0, (double)m_smmap.getMapSizeY()/2.0);
	cv::Point pt1, pt2;
	for (int x = 0; x < m_smmap.getMapSizeX(); x++) {
		for (int y = 0; y < m_smmap.getMapSizeY(); y++) {
			GMapping::IntPoint p(x, y);
			double occ = m_smmap.cell(p);// .mean();
//			assert(occ <= 1.0);
			/*
			_cprintf("smmap.cell(x,y)=%lf\n", m_smmap.cell(p));
			_cprintf("occ(x,y)=%lf\n", occ);
			_cprintf("smmap.cell(x,y).mean=%lf\n", m_smmap.cell(p).mean());
			*/
			pt1.x = x * (GRID_SIZE + LINE_WIDTH);
			pt1.y = (m_smmap.getMapSizeY() - y) * (GRID_SIZE + LINE_WIDTH);
			pt2.x = pt1.x + GRID_SIZE;
			pt2.y = pt1.y - GRID_SIZE;
			double gray_level;
			if (occ < 0) { // Cell = (-1) stands for unknown
				cv::rectangle(m_Mat, pt1, pt2,
					cv::Scalar(127, 127, 127),
					CV_FILLED, 8, 0);
					//_cprintf("(x,y)=%d,%d\n", x, y);
			} else if (occ > occ_thresh_) { // occupied
				cv::rectangle(m_Mat, pt1, pt2,
					cv::Scalar(32, 0, 0),
					CV_FILLED, 8, 0);
					//_cprintf("(x,y)=%d,%d\n", x, y);
			} else { // free
				gray_level = 255.0 * (1 - occ / occ_thresh_);
				cv::rectangle(m_Mat, pt1, pt2,
					//cv::Scalar(255, 255, 255),
					cv::Scalar(gray_level, gray_level, gray_level),
					CV_FILLED, 8, 0);
					//_cprintf("(x,y)=%d,%d\n", x, y);
			}
		}
	}
	
	// draw robot
	//const double SCALE = 20.0;
	cv::Point2d cp;
	//cp.x = (m_Pose.x / delta_ + (m_smmap.getMapSizeX() / 2)) * (GRID_SIZE + LINE_WIDTH) + (GRID_SIZE / 2);
	//cp.y = ((m_smmap.getMapSizeY() / 2) - m_Pose.y / delta_) * (GRID_SIZE + LINE_WIDTH) + (GRID_SIZE / 2);
	cp.x = (4 + m_Pose.x / delta_ + (m_smmap.getMapSizeX() / 2)) * (GRID_SIZE + LINE_WIDTH) + (GRID_SIZE / 2);
	cp.y = ((m_smmap.getMapSizeY() / 2) - m_Pose.y / delta_ - 4) * (GRID_SIZE + LINE_WIDTH) + (GRID_SIZE / 2);
	cv::circle(m_Mat, cp, 10, cv::Scalar(0, 0, 255), CV_FILLED, 8, 0);
	//_cprintf("center(x,y)=%d,%d\n", cp.x, cp.y);
	cv::Point2d dbgCp;
	//cp.x = (m_Pose.x / delta_ + (m_smmap.getMapSizeX() / 2)) * (GRID_SIZE + LINE_WIDTH) + (GRID_SIZE / 2);
	//cp.y = ((m_smmap.getMapSizeY() / 2) - m_Pose.y / delta_) * (GRID_SIZE + LINE_WIDTH) + (GRID_SIZE / 2);
	dbgCp.x = (4 + m_dbgPose.x / delta_ + (m_smmap.getMapSizeX() / 2)) * (GRID_SIZE + LINE_WIDTH) + (GRID_SIZE / 2);
	dbgCp.y = ((m_smmap.getMapSizeY() / 2) - m_dbgPose.y / delta_ - 4) * (GRID_SIZE + LINE_WIDTH) + (GRID_SIZE / 2);
	cv::circle(m_Mat, dbgCp, 10, cv::Scalar(0, 127, 255), 2, 8, 0);

	cv::Point2d _iep;	// IR ray end point (robot frame)
	cv::Point2d iep; // IR ray end point (world frame)
	cv::Point2d ep;	// scale for drawing
	for (size_t i = 0; i < NUM_IR; i++) {
		//_cprintf("ir_readings[%d]: range=%lf, theta=%lf\n", i, ir_readings[i].m_Range, ir_readings[i].m_pose.theta);
		_iep.x = ir_readings[i].m_Range * cos(ir_readings[i].m_pose.theta);
		_iep.y = ir_readings[i].m_Range * sin(ir_readings[i].m_pose.theta);
		//_cprintf("_iep(x,y)=%lf,%lf\n", _iep.x, _iep.y);
		iep.x = m_Pose.x + _iep.x * cos(m_Pose.theta) - _iep.y * sin(m_Pose.theta);
		iep.y = m_Pose.y + _iep.x * sin(m_Pose.theta) + _iep.y * cos(m_Pose.theta);
		//_cprintf("iep(x,y)=%lf,%lf\n", iep.x, iep.y);
		//ep.x = (iep.x / delta_ + (m_smmap.getMapSizeX() / 2)) * (GRID_SIZE + LINE_WIDTH) + (GRID_SIZE / 2);
		//ep.y = ((m_smmap.getMapSizeY() / 2) - iep.y / delta_) * (GRID_SIZE + LINE_WIDTH) + (GRID_SIZE / 2);
		ep.x = (4 + iep.x / delta_ + (m_smmap.getMapSizeX() / 2)) * (GRID_SIZE + LINE_WIDTH) + (GRID_SIZE / 2);
		ep.y = ((m_smmap.getMapSizeY() / 2) - iep.y / delta_ - 4) * (GRID_SIZE + LINE_WIDTH) + (GRID_SIZE / 2);
		cv::line(m_Mat,	ep, cp, cv::Scalar(255, 0, 255), 2, 8, 0);
		//_cprintf("ep(x,y)=%lf,%lf\n", ep.x, ep.y);
	}
	//system("pause");
	/*
	cv::line(m_Mat,
		cv::Point((ROBOT_MAP_SIZE / 2), (ROBOT_MAP_SIZE / 2)),
		cv::Point(m_Pose.x + (ROBOT_MAP_SIZE / 2), (ROBOT_MAP_SIZE / 2) - m_Pose.y),
		cv::Scalar(255, 255, 0), 2, CV_AA);
	cv::line(m_Mat,
		cv::Point(m_Pose.x + (ROBOT_MAP_SIZE / 2), (ROBOT_MAP_SIZE / 2) - m_Pose.y),
		cv::Point((SCALE * cos(m_Pose.theta)) + m_Pose.x + (ROBOT_MAP_SIZE / 2), (ROBOT_MAP_SIZE / 2) - (m_Pose.y + (SCALE * sin(m_Pose.theta)))),
		cv::Scalar(0, 255, 255), 2, CV_AA);
	*/
	cv::imshow("Robot", m_Mat);
}

UINT CX80MultiRobotDlg::ReadDataThreadProc(LPVOID pParam) 
{
	CX80MultiRobotDlg* pCX80Dlg = (CX80MultiRobotDlg*)pParam;
	//_cprintf(pCX80Dlg->m_strFilePath.c_str());
	//pCX80Dlg->m_oFile.open(pCX80Dlg->m_strFilePath.c_str(), std::ios::in);
	
	std::ifstream	file(pCX80Dlg->m_strFilePath.c_str());

	for (CSVIterator loop(file); loop != CSVIterator(); ++loop) {
		_cprintf("[%s] [%s] [%s] [%s] [%s] [%s] [%s] [%s] \n",
			(*loop)[0].c_str(), (*loop)[1].c_str(), (*loop)[2].c_str(), (*loop)[3].c_str(), (*loop)[4].c_str(), (*loop)[5].c_str(), (*loop)[6].c_str(), (*loop)[7].c_str(), (*loop)[8].c_str());
		pCX80Dlg->m_Encoder1 = std::stod((*loop)[0]);
		pCX80Dlg->m_Encoder2 = std::stod((*loop)[1]);
		pCX80Dlg->m_IR1 = std::stod((*loop)[2]);
		pCX80Dlg->m_IR2 = std::stod((*loop)[3]);
		pCX80Dlg->m_IR3 = std::stod((*loop)[4]);
		pCX80Dlg->m_IR4 = std::stod((*loop)[5]);
		pCX80Dlg->m_IR5 = std::stod((*loop)[6]);
		pCX80Dlg->m_IR6 = std::stod((*loop)[7]);
		pCX80Dlg->m_IR7 = std::stod((*loop)[8]);
		pCX80Dlg->UpdateData(false);
		Sleep(20);
	}

	return 0;
}

void CX80MultiRobotDlg::OnBnClickedButtonStart()
{
	// TODO: Add your control notification handler code here
	if (((CButton *)GetDlgItem(IDC_CHECK_SIMULATE))->GetCheck() == true) {
		AfxBeginThread(CX80MultiRobotDlg::ReadDataThreadProc, this);
	}
	m_Encoder1_prev = m_Encoder1;
	m_Encoder2_prev = m_Encoder2;
	m_flagStart = true;
	//system("pause");
	if (((CButton *)GetDlgItem(IDC_CHECK_SAVE_DATA))->GetCheck() == true && 
		((CButton *)GetDlgItem(IDC_CHECK_SIMULATE))->GetCheck() == false) {
		m_oFile.open(m_strFilePath.c_str(), std::ios::out);
	} else {
		SetTimer(ID_TIMER_UPDATE, UPDATE_ELAPSE, NULL);
		SetTimer(ID_TIMER_PROCESS_SCAN, 5 * UPDATE_ELAPSE, NULL);
	}
}
void CX80MultiRobotDlg::OnBnClickedButtonStop()
{
	// TODO: Add your control notification handler code here
	m_flagStart = false;
}

void CX80MultiRobotDlg::OnBnClickedButtonTest()
{
	// TODO: Add your control notification handler code here

	AfxBeginThread(MyThreadProc, this);
	//MessageBox("Leave OnBnClickedButtonTest");

	/*/ qiao@2015.08.07 opencv test
	cv::Mat image;
	image = cv::imread("D:\\Users\\qianhao_huang.ASUS\\Desktop\\test.png", cv::IMREAD_COLOR); // Read the file
	// Check for invalid input
	if (image.empty()) { MessageBox("Could not open or find the image"); }

	cv::namedWindow("Display window", cv::WINDOW_AUTOSIZE); // Create a window for display.
	imshow("Display window", image); // Show our image inside it.

	cv::waitKey(0); // Wait for a keystroke in the window
	*/ // opencv test end

	// qiao@2015.08.10 add timer
	//m_Mat = cv::Mat(200, 200, CV_8UC3, cv::Scalar(0));
	
	// init m_Pose
	//_cprintf("Test\n");
}

void CX80MultiRobotDlg::OnDeltaposSpinVelocity1(NMHDR *pNMHDR, LRESULT *pResult)
{
	LPNMUPDOWN pNMUpDown = reinterpret_cast<LPNMUPDOWN>(pNMHDR);
	// TODO: Add your control notification handler code here
	pNMUpDown->iDelta > 0 ? m_VelocityL++ : m_VelocityL--;
	UpdateData(false);
	*pResult = 0;
}

void CX80MultiRobotDlg::OnDeltaposSpinVelocity2(NMHDR *pNMHDR, LRESULT *pResult)
{
	LPNMUPDOWN pNMUpDown = reinterpret_cast<LPNMUPDOWN>(pNMHDR);
	// TODO: Add your control notification handler code here
	pNMUpDown->iDelta > 0 ? m_VelocityR-- : m_VelocityR++;
	UpdateData(false);
	*pResult = 0;
}

void CX80MultiRobotDlg::OnEnChangeVelocity1()
{
	// TODO:  If this is a RICHEDIT control, the control will not
	// send this notification unless you override the CDialog::OnInitDialog()
	// function and call CRichEditCtrl().SetEventMask()
	// with the ENM_CHANGE flag ORed into the mask.

	// TODO:  Add your control notification handler code here
	UpdateData(true);
}

void CX80MultiRobotDlg::OnEnChangeVelocity2()
{
	// TODO:  If this is a RICHEDIT control, the control will not
	// send this notification unless you override the CDialog::OnInitDialog()
	// function and call CRichEditCtrl().SetEventMask()
	// with the ENM_CHANGE flag ORed into the mask.

	// TODO:  Add your control notification handler code here
	UpdateData(true);
}

void CX80MultiRobotDlg::OnBnClickedButtonReset()
{
	// TODO: Add your control notification handler code here
	m_Pose.x = 0.0;
	m_Pose.y = 0.0;
	m_Pose.theta = 0.0;// CONST_PI / 2.0;

	m_count = 0;
	m_flagStart = false;
	cv::destroyAllWindows();
	KillTimer(ID_TIMER_UPDATE);
	KillTimer(ID_TIMER_PROCESS_SCAN);

	RobotParamInit();
}
