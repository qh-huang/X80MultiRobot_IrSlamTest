// X80MultiRobot.cpp : Defines the class behaviors for the application.
//

#include "stdafx.h"
#include "X80MultiRobot.h"
#include "X80MultiRobotDlg.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#undef THIS_FILE
static char THIS_FILE[] = __FILE__;
#endif

/////////////////////////////////////////////////////////////////////////////
// CX80MultiRobotApp

BEGIN_MESSAGE_MAP(CX80MultiRobotApp, CWinApp)
	//{{AFX_MSG_MAP(CX80MultiRobotApp)
		// NOTE - the ClassWizard will add and remove mapping macros here.
		//    DO NOT EDIT what you see in these blocks of generated code!
	//}}AFX_MSG
	ON_COMMAND(ID_HELP, CWinApp::OnHelp)
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CX80MultiRobotApp construction

CX80MultiRobotApp::CX80MultiRobotApp()
{
	// TODO: add construction code here,
	// Place all significant initialization in InitInstance
}

/////////////////////////////////////////////////////////////////////////////
// The one and only CX80MultiRobotApp object

CX80MultiRobotApp theApp;

/////////////////////////////////////////////////////////////////////////////
// CX80MultiRobotApp initialization

BOOL CX80MultiRobotApp::InitInstance()
{
	AfxEnableControlContainer();

	// Standard initialization
	// If you are not using these features and wish to reduce the size
	//  of your final executable, you should remove from the following
	//  the specific initialization routines you do not need.

//#ifdef DBUG // qiao@2015.08.24: add debug console
	if (!AllocConsole())
		AfxMessageBox("Failed to create the console!", MB_ICONEXCLAMATION);
//#endif

#ifdef _AFXDLL
	//Enable3dControls();			// Call this when using MFC in a shared DLL
#else
	//Enable3dControlsStatic();	// Call this when linking to MFC statically
#endif

	CX80MultiRobotDlg dlg;
	m_pMainWnd = &dlg;
	int nResponse = dlg.DoModal();
	if (nResponse == IDOK)
	{
		// TODO: Place code here to handle when the dialog is
		//  dismissed with OK
	}
	else if (nResponse == IDCANCEL)
	{
		// TODO: Place code here to handle when the dialog is
		//  dismissed with Cancel
	}

	// Since the dialog has been closed, return FALSE so that we exit the
	//  application, rather than start the application's message pump.
	return FALSE;
}

int CX80MultiRobotApp::ExitInstance() 
{
	//	deallocate console
//#ifdef _DEBUG
	if (!FreeConsole())
		AfxMessageBox("Could not free the console!");
//#endif
	return CWinApp::ExitInstance();
}
