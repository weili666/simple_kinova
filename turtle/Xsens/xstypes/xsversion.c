/*	WARNING: COPYRIGHT (C) 2016 XSENS TECHNOLOGIES OR SUBSIDIARIES WORLDWIDE. ALL RIGHTS RESERVED.
	THIS FILE AND THE SOURCE CODE IT CONTAINS (AND/OR THE BINARY CODE FILES FOUND IN THE SAME
	FOLDER THAT CONTAINS THIS FILE) AND ALL RELATED SOFTWARE (COLLECTIVELY, "CODE") ARE SUBJECT
	TO A RESTRICTED LICENSE AGREEMENT ("AGREEMENT") BETWEEN XSENS AS LICENSOR AND THE AUTHORIZED
	LICENSEE UNDER THE AGREEMENT. THE CODE MUST BE USED SOLELY WITH XSENS PRODUCTS INCORPORATED
	INTO LICENSEE PRODUCTS IN ACCORDANCE WITH THE AGREEMENT. ANY USE, MODIFICATION, COPYING OR
	DISTRIBUTION OF THE CODE IS STRICTLY PROHIBITED UNLESS EXPRESSLY AUTHORIZED BY THE AGREEMENT.
	IF YOU ARE NOT AN AUTHORIZED USER OF THE CODE IN ACCORDANCE WITH THE AGREEMENT, YOU MUST STOP
	USING OR VIEWING THE CODE NOW, REMOVE ANY COPIES OF THE CODE FROM YOUR COMPUTER AND NOTIFY
	XSENS IMMEDIATELY BY EMAIL TO INFO@XSENS.COM. ANY COPIES OR DERIVATIVES OF THE CODE (IN WHOLE
	OR IN PART) IN SOURCE CODE FORM THAT ARE PERMITTED BY THE AGREEMENT MUST RETAIN THE ABOVE
	COPYRIGHT NOTICE AND THIS PARAGRAPH IN ITS ENTIRETY, AS REQUIRED BY THE AGREEMENT.
*/

#include "xsversion.h"
#include "xsstring.h"
#include <stdio.h>

/*! \class XsVersion
	\brief A class to store version information
*/

/*! \addtogroup cinterface C Interface
	@{
*/

/*! \relates XsVersion \brief Test if this is a null-version. */
int XsVersion_empty(const XsVersion* thisPtr)
{
	return thisPtr->m_major == 0 && thisPtr->m_minor == 0 && thisPtr->m_revision == 0;
}

/*! \relates XsVersion \brief Get a string with the version expressed in a readable format. */
void XsVersion_toString(const XsVersion* thisPtr, XsString* version)
{
	char buffer[256];
	size_t chars;

	if (XsVersion_empty(thisPtr))
		return;

	if (thisPtr->m_build != 0 && thisPtr->m_reposVersion != 0)
		chars = sprintf(buffer, "%d.%d.%d build %d rev %d", thisPtr->m_major, thisPtr->m_minor, thisPtr->m_revision, thisPtr->m_build, thisPtr->m_reposVersion);
	else
		chars = sprintf(buffer, "%d.%d.%d", thisPtr->m_major, thisPtr->m_minor, thisPtr->m_revision);
	
	XsString_assign(version, chars, buffer);
	if (thisPtr->m_extra.m_size != 0)
	{
		const char space = ' ';
		XsArray_insert(version, version->m_size-1, 1, &space);	//lint !e64
		XsString_append(version, &thisPtr->m_extra);
	}
}

/*! @} */
