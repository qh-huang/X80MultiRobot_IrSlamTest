// X80MultiRobotDlg.h : header file
//
//{{AFX_INCLUDES()
#include "wirobotsdk.h"
//}}AFX_INCLUDES
#include <opencv2/opencv.hpp>
//#include "CPose2D.h"
#include "include/gmapping/utils/GPoint.h"
#include "include/gmapping/scanmatcher/scanmatcher.h"
#include "include/gmapping/scanmatcher/smmap.h"
#include <fstream>

#if !defined(AFX_X80MULTIROBOTDLG_H__804EE31C_32BE_4316_8F38_84907F57A737__INCLUDED_)
#define AFX_X80MULTIROBOTDLG_H__804EE31C_32BE_4316_8F38_84907F57A737__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

/////////////////////////////////////////////////////////////////////////////
// CX80MultiRobotDlg dialog

class CX80MultiRobotDlg : public CDialog
{
// Construction
public:
	bool musicInPlay;
	CX80MultiRobotDlg(CWnd* pParent = NULL);	// standard constructor
	~CX80MultiRobotDlg();

// Dialog Data
	//{{AFX_DATA(CX80MultiRobotDlg)
	enum { IDD = IDD_X80MULTIROBOT_DIALOG };
	CButton	m_playMusic;
	CWiRobotSDK	m_MOTSDK;
	short	m_Encoder1;
	short	m_Encoder2;
	short	m_IR1;
	short	m_IR2;
	short	m_IR3;
	short	m_IR4;
	short	m_IR5;
	short	m_IR6;
	short	m_IR7;
	short	m_Sonar1;
	short	m_Sonar2;
	short	m_Sonar3;
	CWiRobotSDK	m_TISDK;
	//}}AFX_DATA

	// qiao
	//CString m_Log;
	UINT	m_IR_1;
	UINT	m_IR_2;
	UINT	m_IR_3;
	UINT	m_IR_4;
	UINT	m_IR_5;
	UINT	m_IR_6;
	UINT	m_IR_7;
	UINT	m_Sonar_1;
	UINT	m_Sonar_2;
	UINT	m_Sonar_3;
	cv::Mat m_Mat;
	double	m_VelocityL;
	double	m_VelocityR;
	bool m_Moving;
	//mrpt::poses::CPose2D m_Pose;	// robot pose
	GMapping::OrientedPoint m_Pose;
	GMapping::OrientedPoint m_dbgPose;	// debug only
	double m_Encoder1_prev;
	double m_Encoder2_prev;
	GMapping::ScanMatcher m_matcher;
	//bool m_got_first_scan_;

	void RobotParamInit();
	void RobotSensorUpdate();
	void RobotPoseUpdate_Odometry();
	void RobotPoseUpdate_ProcessScan();
	void RobotDebugCV();

	int m_count;
	double m_minimumScore;
	GMapping::ScanMatcherMap m_smmap;

	double delta_;
	double occ_thresh_;

	bool m_flagStart;
	std::string m_strFilePath;
	std::fstream m_oFile;
	static UINT ReadDataThreadProc(LPVOID pParam);

	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CX80MultiRobotDlg)
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV support
	//}}AFX_VIRTUAL

// Implementation
protected:
	HICON m_hIcon;

	BOOL PreTranslateMessage(MSG * pMsg);

	//VOID OnKeyDown(UINT nChar, UINT nRepCnt, UINT nFlags);

	// Generated message map functions
	//{{AFX_MSG(CX80MultiRobotDlg)
	virtual BOOL OnInitDialog();
	//afx_msg void OnKeyDown(UINT nChar, UINT nRepCnt, UINT nFlags);
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	afx_msg void OnStandardSensorEventDrrobotsdkcontrolctrl1();
	afx_msg void OnCustomSensorEventDrrobotsdkcontrolctrl1();
	afx_msg void OnMotorSensorEventDrrobotsdkcontrolctrl1();
	afx_msg void OnForward();
	afx_msg void OnBack();
	afx_msg void OnStop();
	afx_msg void OnTurnLeft();
	afx_msg void OnTurnRight();
	afx_msg void OnPlayAudio();
	virtual void OnCancel();
	afx_msg void OnTimer(UINT nIDEvent);
	afx_msg void OnRecord();
	afx_msg void OnVoiceSegmentEventDrrobotsdkcontrolctrl1();
	afx_msg void OnVoiceSegmentEventDrrobotsdkcontrolctrl2();
	//
	afx_msg void OnBnClickedButtonTest();
	//afx_msg void OnKeyDown(UINT nChar, UINT nRepCnt, UINT nFlags);
	DECLARE_EVENTSINK_MAP()
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnDeltaposSpinVelocity1(NMHDR *pNMHDR, LRESULT *pResult);
	afx_msg void OnDeltaposSpinVelocity2(NMHDR *pNMHDR, LRESULT *pResult);
	afx_msg void OnEnChangeVelocity1();
	afx_msg void OnEnChangeVelocity2();
	afx_msg void OnBnClickedButtonReset();
	afx_msg void OnBnClickedButtonStart();
	afx_msg void OnBnClickedButtonStop();
	afx_msg void OnBnClickedButtonSelectFilePath();
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_X80MULTIROBOTDLG_H__804EE31C_32BE_4316_8F38_84907F57A737__INCLUDED_)
