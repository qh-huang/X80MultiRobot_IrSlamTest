// X80MultiRobotDlg.cpp : implementation file
//

#include "stdafx.h"
#include "X80MultiRobot.h"
#include "X80MultiRobotDlg.h"
//#include <opencv2/opencv.hpp>
#include <cmath>
//#include "CPoint2D.h"
#include <cstring>
#include <vector>
#include <iostream>
#include <fstream>
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


BOOL SaveAudioFile(LPCTSTR FileName) ;
struct WAVEFileHeader{
	short wFormatTag;
	short nChannels;
	long nSamplesPerSec;
	long nAvgBytesPerSec;
	short nBlockAlign;
	short wBitsPerSample;
	short cbSize;
}waveFormatHeader;

struct  FileHeader
{
	long lRiff;
	long lFileSize;
	long lWave ;
	long lFormat;
	long lFormatLength;
} audioFileHeader;

struct ChunkHeader
{
  long lType;
  long lLen;
}ch;

class CAboutDlg : public CDialog
{
public:
	CAboutDlg();

// Dialog Data
	//{{AFX_DATA(CAboutDlg)
	enum { IDD = IDD_ABOUTBOX };
	//}}AFX_DATA

	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CAboutDlg)
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
	//}}AFX_VIRTUAL

// Implementation
protected:
	//{{AFX_MSG(CAboutDlg)
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialog(CAboutDlg::IDD)
{
	//{{AFX_DATA_INIT(CAboutDlg)
	//}}AFX_DATA_INIT
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	//{{AFX_DATA_MAP(CAboutDlg)
	//}}AFX_DATA_MAP
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialog)
	//{{AFX_MSG_MAP(CAboutDlg)
		// No message handlers
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CX80MultiRobotDlg dialog

CX80MultiRobotDlg::CX80MultiRobotDlg(CWnd* pParent /*=NULL*/)
	: CDialog(CX80MultiRobotDlg::IDD, pParent)
{
	//{{AFX_DATA_INIT(CX80MultiRobotDlg)
	m_Encoder1 = 0;
	m_Encoder2 = 0;
	m_IR1 = 0;
	m_IR2 = 0;
	m_IR3 = 0;
	m_IR4 = 0;
	m_IR5 = 0;
	m_IR6 = 0;
	m_IR7 = 0;
	m_Sonar1 = 0;
	m_Sonar2 = 0;
	m_Sonar3 = 0;
	//}}AFX_DATA_INIT
	// Note that LoadIcon does not require a subsequent DestroyIcon in Win32
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);

	// qiao@2015.08.11
	RobotParamInit();	
}

void CX80MultiRobotDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	//{{AFX_DATA_MAP(CX80MultiRobotDlg)
	//DDX_Control(pDX, IDC_PlayAudio, m_playMusic);
	DDX_Control(pDX, IDC_DRROBOTSDKCONTROLCTRL1, m_MOTSDK);
	DDX_Text(pDX, IDC_Encoder1, m_Encoder1);
	DDX_Text(pDX, IDC_Encoder2, m_Encoder2);
	DDX_Text(pDX, IDC_IR1, m_IR1);
	DDX_Text(pDX, IDC_IR2, m_IR2);
	DDX_Text(pDX, IDC_IR3, m_IR3);
	DDX_Text(pDX, IDC_IR4, m_IR4);
	DDX_Text(pDX, IDC_IR5, m_IR5);
	DDX_Text(pDX, IDC_IR6, m_IR6);
	DDX_Text(pDX, IDC_IR7, m_IR7);
	DDX_Text(pDX, IDC_Sonar1, m_Sonar1);
	DDX_Text(pDX, IDC_Sonar2, m_Sonar2);
	DDX_Text(pDX, IDC_Sonar3, m_Sonar3);
	// qiao@2015.08.10
	DDX_Text(pDX, IDC_IR_1, m_IR_1);
	DDX_Text(pDX, IDC_IR_2, m_IR_2);
	DDX_Text(pDX, IDC_IR_3, m_IR_3);
	DDX_Text(pDX, IDC_IR_4, m_IR_4);
	DDX_Text(pDX, IDC_IR_5, m_IR_5);
	DDX_Text(pDX, IDC_IR_6, m_IR_6);
	DDX_Text(pDX, IDC_IR_7, m_IR_7);
	DDX_Text(pDX, IDC_Sonar_1, m_Sonar1);
	DDX_Text(pDX, IDC_Sonar_2, m_Sonar2);
	DDX_Text(pDX, IDC_Sonar_3, m_Sonar3);
	// end qiao
	DDX_Control(pDX, IDC_DRROBOTSDKCONTROLCTRL2, m_TISDK);
	//DDX_Text(pDX, IDC_STATIC_LOG, m_Log);
	DDX_Text(pDX, IDC_Velocity1, m_VelocityL);
	DDX_Text(pDX, IDC_Velocity2, m_VelocityR);
	DDX_Text(pDX, IDC_STATIC_FILE_PATH, (CString)m_strFilePath.c_str());
	//}}AFX_DATA_MAP
}

BEGIN_MESSAGE_MAP(CX80MultiRobotDlg, CDialog)
	//{{AFX_MSG_MAP(CX80MultiRobotDlg)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_BN_CLICKED(IDC_Forward, OnForward)
	ON_BN_CLICKED(IDC_Back, OnBack)
	ON_BN_CLICKED(IDC_Stop, OnStop)
	ON_BN_CLICKED(IDC_TurnLeft, OnTurnLeft)
	ON_BN_CLICKED(IDC_TurnRight, OnTurnRight)
	ON_WM_TIMER()
	//}}AFX_MSG_MAP
	ON_BN_CLICKED(IDC_BUTTON_TEST, OnBnClickedButtonTest)
	ON_NOTIFY(UDN_DELTAPOS, IDC_SPIN_Velocity1, &CX80MultiRobotDlg::OnDeltaposSpinVelocity1)
	ON_NOTIFY(UDN_DELTAPOS, IDC_SPIN_Velocity2, &CX80MultiRobotDlg::OnDeltaposSpinVelocity2)
	//ON_WM_KEYDOWN(WM_KEYDOWN, OnKeyDown)
	ON_EN_CHANGE(IDC_Velocity1, &CX80MultiRobotDlg::OnEnChangeVelocity1)
	ON_EN_CHANGE(IDC_Velocity2, &CX80MultiRobotDlg::OnEnChangeVelocity2)
	ON_BN_CLICKED(IDC_BUTTON_RESET_POSE, &CX80MultiRobotDlg::OnBnClickedButtonReset)
	ON_BN_CLICKED(IDC_BUTTON_START, &CX80MultiRobotDlg::OnBnClickedButtonStart)
	ON_BN_CLICKED(IDC_BUTTON_STOP, &CX80MultiRobotDlg::OnBnClickedButtonStop)
	ON_BN_CLICKED(IDC_BUTTON_SELECT_FILE_PATH, &CX80MultiRobotDlg::OnBnClickedButtonSelectFilePath)
END_MESSAGE_MAP()

BOOL CX80MultiRobotDlg::OnInitDialog()
{
	CDialog::OnInitDialog();

	// Add "About..." menu item to system menu.

	// IDM_ABOUTBOX must be in the system command range.
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != NULL)
	{
		CString strAboutMenu;
		strAboutMenu.LoadString(IDS_ABOUTBOX);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// Set the icon for this dialog.  The framework does this automatically
	//  when the application's main window is not a dialog
	SetIcon(m_hIcon, TRUE);			// Set big icon
	SetIcon(m_hIcon, FALSE);		// Set small icon
	
	// TODO: Add extra initialization here
	
	m_MOTSDK.connectRobot ("drrobot1");
	//m_TISDK.connectRobot ("drrobot2");			//just for demo to add another activex control
	
	musicInPlay = false;
	SetTimer(1, 450, NULL);

	return TRUE;  // return TRUE  unless you set the focus to a control
}

void CX80MultiRobotDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialog::OnSysCommand(nID, lParam);
	}
}

// If you add a minimize button to your dialog, you will need the code below
//  to draw the icon.  For MFC applications using the document/view model,
//  this is automatically done for you by the framework.

void CX80MultiRobotDlg::OnPaint() 
{
	if (IsIconic())
	{
		CPaintDC dc(this); // device context for painting

		SendMessage(WM_ICONERASEBKGND, (WPARAM) dc.GetSafeHdc(), 0);

		// Center icon in client rectangle
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// Draw the icon
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialog::OnPaint();
	}
}

// The system calls this to obtain the cursor to display while the user drags
//  the minimized window.
HCURSOR CX80MultiRobotDlg::OnQueryDragIcon()
{
	return (HCURSOR) m_hIcon;
}

BEGIN_EVENTSINK_MAP(CX80MultiRobotDlg, CDialog)
    //{{AFX_EVENTSINK_MAP(CX80MultiRobotDlg)
	ON_EVENT(CX80MultiRobotDlg, IDC_DRROBOTSDKCONTROLCTRL1, 1 /* StandardSensorEvent */, OnStandardSensorEventDrrobotsdkcontrolctrl1, VTS_NONE)
	ON_EVENT(CX80MultiRobotDlg, IDC_DRROBOTSDKCONTROLCTRL1, 3 /* CustomSensorEvent */, OnCustomSensorEventDrrobotsdkcontrolctrl1, VTS_NONE)
	ON_EVENT(CX80MultiRobotDlg, IDC_DRROBOTSDKCONTROLCTRL1, 2 /* MotorSensorEvent */, OnMotorSensorEventDrrobotsdkcontrolctrl1, VTS_NONE)
	//ON_EVENT(CX80MultiRobotDlg, IDC_DRROBOTSDKCONTROLCTRL1, 5 /* VoiceSegmentEvent */, OnVoiceSegmentEventDrrobotsdkcontrolctrl1, VTS_NONE)
	//ON_EVENT(CX80MultiRobotDlg, IDC_DRROBOTSDKCONTROLCTRL2, 5 /* VoiceSegmentEvent */, OnVoiceSegmentEventDrrobotsdkcontrolctrl2, VTS_NONE)
	//}}AFX_EVENTSINK_MAP
END_EVENTSINK_MAP()

void CX80MultiRobotDlg::OnStandardSensorEventDrrobotsdkcontrolctrl1() 
{
	// TODO: Add your control notification handler code here
	m_Sonar1= m_MOTSDK.GetSensorSonar1 ();
	m_Sonar2= m_MOTSDK.GetSensorSonar2 ();
	m_Sonar3= m_MOTSDK.GetSensorSonar3 ();

	m_IR1 = m_MOTSDK.GetSensorIRRange ();	

	UpdateData(false);

	// qiao@2015.08.11
	//IR1_Array.add(m_IR1);
}

void CX80MultiRobotDlg::OnCustomSensorEventDrrobotsdkcontrolctrl1() 
{
	// TODO: Add your control notification handler code here
	m_IR2 = m_MOTSDK.GetCustomAD3 ();
	m_IR3 = m_MOTSDK.GetCustomAD4 ();
	m_IR4 = m_MOTSDK.GetCustomAD5 ();
	m_IR5 = m_MOTSDK.GetCustomAD6 ();
	m_IR6 = m_MOTSDK.GetCustomAD7 ();
	m_IR7 = m_MOTSDK.GetCustomAD8 ();
	UpdateData(false);

	/* qiao@2015.08.11
	IR2_Array.add(m_IR2);
	IR3_Array.add(m_IR3);
	IR4_Array.add(m_IR4);
	IR5_Array.add(m_IR5);
	IR6_Array.add(m_IR6);
	IR7_Array.add(m_IR7);
	*/
}

void CX80MultiRobotDlg::OnMotorSensorEventDrrobotsdkcontrolctrl1() 
{
	// TODO: Add your control notification handler code here
	m_Encoder1 = m_MOTSDK.GetEncoderPulse1 ();
	m_Encoder2 = m_MOTSDK.GetEncoderPulse2 ();
	UpdateData(false);

	// qiao@2015.08.31: save data
	if (((CButton *)GetDlgItem(IDC_CHECK_SAVE_DATA))->GetCheck() == true && 
		((CButton *)GetDlgItem(IDC_CHECK_SIMULATE))->GetCheck() == false &&
		m_oFile.is_open()) {
		/*
		std::stringstream ss;
		ss << m_Encoder1 << "," << m_Encoder2 << ","
			<< m_IR1 << "," << m_IR2 << "," << m_IR3 << "," << m_IR4 << "," << m_IR5 << "," << m_IR6 << "," << m_IR7
			<< std::endl;
		_cprintf(ss.str().c_str());
		*/
		m_oFile << m_Encoder1 << "," << m_Encoder2 << ","
			<< m_IR1 << "," << m_IR2 << "," << m_IR3 << "," << m_IR4 << "," << m_IR5 << "," << m_IR6 << "," << m_IR7
			<< std::endl;
	}
}

void CX80MultiRobotDlg::OnForward() 
{
	// TODO: Add your control notification handler code here
	m_MOTSDK.SetDcMotorControlMode (0,M_VELOCITY);
	m_MOTSDK.SetDcMotorControlMode (1,M_VELOCITY);
	m_MOTSDK.SetDcMotorVelocityControlPID (0, 30, 10, 0);
	m_MOTSDK.SetDcMotorVelocityControlPID (1, 30, 10, 0);
	//m_MOTSDK.DcMotorVelocityNonTimeCtrAll (-200, 200,NO_CONTROL,NO_CONTROL,NO_CONTROL,NO_CONTROL);
	m_MOTSDK.DcMotorVelocityNonTimeCtrAll(-m_VelocityL, m_VelocityR, NO_CONTROL, NO_CONTROL, NO_CONTROL, NO_CONTROL);
}

void CX80MultiRobotDlg::OnBack() 
{
	// TODO: Add your control notification handler code here
	m_MOTSDK.SetDcMotorControlMode (0,M_VELOCITY);
	m_MOTSDK.SetDcMotorControlMode (1,M_VELOCITY);
	m_MOTSDK.SetDcMotorVelocityControlPID (0, 30, 10, 0);
	m_MOTSDK.SetDcMotorVelocityControlPID (1, 30, 10, 0);
	m_MOTSDK.DcMotorVelocityNonTimeCtrAll (200, -200,NO_CONTROL,NO_CONTROL,NO_CONTROL,NO_CONTROL); 
}

void CX80MultiRobotDlg::OnStop() 
{
	// TODO: Add your control notification handler code here
	
	m_MOTSDK.DisableDcMotor (0);
	m_MOTSDK.DisableDcMotor (1);
}

void CX80MultiRobotDlg::OnTurnLeft() 
{
	// TODO: Add your control notification handler code here
	long cmd1,cmd2;

    cmd1 = m_Encoder1 + cWHOLE_RANGE / 4;
    cmd2 = m_Encoder2 + cWHOLE_RANGE / 4;
    
   //change cmd1, cmd2 to valid data range
    if (cmd1 < 0) cmd1 = cmd1 + cFULL_COUNT;
    if (cmd2 < 0) cmd2 = cmd2 + cFULL_COUNT;
    if (cmd1 > cFULL_COUNT) cmd1 = cmd1 - cFULL_COUNT;
    if (cmd2 > cFULL_COUNT) cmd2 = cmd2 - cFULL_COUNT;


	m_MOTSDK.SetDcMotorControlMode (0,M_POSITION);
	m_MOTSDK.SetDcMotorControlMode (1,M_POSITION);
	m_MOTSDK.SetDcMotorVelocityControlPID (0, 30, 10, 0);
	m_MOTSDK.SetDcMotorPositionControlPID (0, 600,30,600);
	m_MOTSDK.SetDcMotorPositionControlPID (1, 600,30,600);
	m_MOTSDK.DcMotorPositionTimeCtrAll (cmd1,cmd2,NO_CONTROL,NO_CONTROL,NO_CONTROL,NO_CONTROL,1000);

}

void CX80MultiRobotDlg::OnTurnRight() 
{
	// TODO: Add your control notification handler code here
	long cmd1,cmd2;

    cmd1 = m_Encoder1 - cWHOLE_RANGE / 4;
    cmd2 = m_Encoder2 - cWHOLE_RANGE / 4;
    
    // change cmd1, cmd2 to valid data range
    if (cmd1 < 0) cmd1 = cmd1 + cFULL_COUNT;
    if (cmd2 < 0) cmd2 = cmd2 + cFULL_COUNT;
    if (cmd1 > cFULL_COUNT) cmd1 = cmd1 - cFULL_COUNT;
    if (cmd2 > cFULL_COUNT) cmd2 = cmd2 - cFULL_COUNT;


	m_MOTSDK.SetDcMotorControlMode (0,M_POSITION);
	m_MOTSDK.SetDcMotorControlMode (1,M_POSITION);
	m_MOTSDK.SetDcMotorVelocityControlPID (0, 30, 10, 0);
	m_MOTSDK.SetDcMotorPositionControlPID (0, 600,30,600);
	m_MOTSDK.SetDcMotorPositionControlPID (1, 600,30,600);
	m_MOTSDK.DcMotorPositionTimeCtrAll (cmd1,cmd2,NO_CONTROL,NO_CONTROL,NO_CONTROL,NO_CONTROL,1000);	
}

void CX80MultiRobotDlg::OnPlayAudio() 
{
	// TODO: Add your control notification handler code here
	char buffer[200];
	
	if (!musicInPlay)
	{ 
		musicInPlay = TRUE;
	
		m_playMusic.SetWindowText ("Stop Music");
		
		::GetCurrentDirectory(200,buffer);
		strcat_s(buffer,"\\happy8k.wav");

		m_MOTSDK.PlayAudioFile(buffer);
		
	} else
	{
	m_MOTSDK.StopAudioPlay ();
	m_playMusic.SetWindowText ("Play Music");
	musicInPlay = FALSE;
	
	}	
}

void CX80MultiRobotDlg::OnCancel() 
{
	// TODO: Add extra cleanup here
	m_MOTSDK.StopAudioPlay ();
	m_MOTSDK.DisableDcMotor (0);
	m_MOTSDK.DisableDcMotor (1);
		
	CDialog::OnCancel();
}

void CX80MultiRobotDlg::OnRecord() 
{
	// TODO: Add your control notification handler code here
	m_MOTSDK.StartRecord (4);
}

void CX80MultiRobotDlg::OnVoiceSegmentEventDrrobotsdkcontrolctrl1() 
{
	// TODO: Add your control notification handler code here
	short voiceLength;
	char	*voiceData;

	voiceData = (char *)m_MOTSDK.GetVoiceSegment ();
	voiceLength = m_MOTSDK.GetVoiceSegLength ();
	
	char lpAudioFile[64] ;
	sprintf_s(lpAudioFile, "c:\\AUDIO1.wav");

	audioFileHeader.lRiff = 0x46464952;  //RIFF
	audioFileHeader.lFileSize = 41006;//sizeof(audioFileHeader) + sizeof(waveFormatHeader) + sizeof(ch) + audioBufIndex ;   
	audioFileHeader.lWave = 0x45564157;			//WAVE
	audioFileHeader.lFormat = 0x20746D66;		//.fmt
	audioFileHeader.lFormatLength = sizeof(waveFormatHeader);

	
	waveFormatHeader.wFormatTag = 1;            
	waveFormatHeader.nChannels = 1;              
	waveFormatHeader.nSamplesPerSec = 8000;      
	waveFormatHeader.wBitsPerSample = 16;        
	waveFormatHeader.nBlockAlign = 2;            
	waveFormatHeader.nAvgBytesPerSec = 16000;     
	waveFormatHeader.cbSize = 0;

	ch.lType = 0x61746164;  
	ch.lLen =  voiceLength;

   
	//FILE *  audiofile = fopen_s(lpAudioFile, "wb");
	FILE* audiofile;
	fopen_s(&audiofile, lpAudioFile, "wb");

	fwrite(&audioFileHeader, 1,sizeof(audioFileHeader),audiofile);
	//fwrite(&audioFileHeader, 1,20,audiofile);
	fwrite(&waveFormatHeader, 1,sizeof(waveFormatHeader),audiofile); 
	//fwrite(&waveFormatHeader, 1,18,audiofile); 
	fwrite(&ch, 1,sizeof(ch),audiofile); 	
	//fwrite(&ch, 1,8,audiofile); 	

	fwrite( voiceData, sizeof(char), voiceLength, audiofile );   

    fclose(audiofile);

}

void CX80MultiRobotDlg::OnVoiceSegmentEventDrrobotsdkcontrolctrl2() 
{
	// TODO: Add your control notification handler code here
	
}

BOOL SaveAudioFile(LPCTSTR FileName) { return TRUE; }

void CX80MultiRobotDlg::OnBnClickedButtonSelectFilePath()
{
	// TODO: Add your control notification handler code here
	char  strFilter[] = { "CSV Files (*.csv)|*.csv|" };
	CFileDialog dlgFile(FALSE, CString(".csv"), NULL, OFN_EXPLORER | OFN_HIDEREADONLY, CString(strFilter));
	if (dlgFile.DoModal() == IDOK) {
		m_strFilePath = dlgFile.GetPathName();
		UpdateData(false);
		//MessageBox(dlgFile.GetPathName());
		/* to get file name only, add following
		int nPos = cstr.ReverseFind(_T('\\'));
		if (nPos > -1)
			m_strFilePath = cstr.Left(nPos + 1);
		*/
	}
}
