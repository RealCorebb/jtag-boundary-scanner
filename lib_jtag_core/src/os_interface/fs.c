/*
 * JTAG Core library
 * Copyright (c) 2008 - 2023 Viveris Technologies
 *
 * JTAG Core library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * JTAG Core library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with JTAG Core library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 */

/**
* @file   fs.c
* @brief  Basic/generic OS file system functions wrapper.
* @author Jean-Fran�ois DEL NERO <Jean-Francois.DELNERO@viveris.fr>
*/

#include <assert.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <limits.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <time.h>

#include "stdint.h"

#ifdef	WIN32
#	include <windows.h>
#	include <io.h>
#	include <fcntl.h>
#	include <direct.h>
#else
#	include <unistd.h>
#	include <dirent.h>

#ifdef	OSX
#	include <mach-o/dyld.h>
#endif

#endif

#include "../jtag_core.h"

#include "os_interface.h"


#ifdef WIN32


int	convertpath	(const char * path, wchar_t * wpath)
{
	if (!MultiByteToWideChar (CP_UTF8, 0, path, -1, wpath, MAX_PATH))
	{
		errno = ENOENT;
		return -1;
	}
	wpath[MAX_PATH] = L'\0';
	return 0;
}
#else

#endif


int genos_open (const char *filename, int flags, ...)
{

#ifdef WIN32
	wchar_t wpath[MAX_PATH+1];
#else
	const char *local_name;
#endif

	unsigned int mode = 0;
	va_list ap;

#if defined (DEBUG)
	printf("genos_open : filename=%s flags=%x\n",filename,flags);
#endif

	va_start (ap, flags);
	if (flags & O_CREAT)
		mode = va_arg (ap, unsigned int);
	va_end (ap);

#ifdef WIN32

	memset(wpath,0,(MAX_PATH+1)*sizeof(wchar_t));
	if(convertpath(filename, wpath)<0)
		return -1;

	return _wopen (wpath, flags, mode);

#else

	local_name = filename;
	//local_name = (const char *)ToLocale (filename);

	if (local_name == NULL)
	{
		errno = ENOENT;
		return -1;
	}

	int fd = open (local_name, flags, mode);

	//LocaleFree (local_name);
	return fd;

#endif


}

FILE *genos_fopen (const char *filename, const char *mode)
{
	int rwflags, oflags;
	int append;
	int fd;
	const char *ptr;
	FILE *stream;

#if defined (DEBUG)
	printf("genos_fopen : filename=%s mode=%s\n",filename,mode);
#endif

	// Try the classic way (ascii path)
	stream = fopen(filename,mode);

	if( ( stream == NULL ) && (errno == ENOENT) )
	{
		// Try the utf8 way
		rwflags = 0;
		oflags = 0;
		append = 0;

		for(ptr = mode;*ptr;ptr++)
		{
			switch (*ptr)
			{
				case 'r':
					rwflags = O_RDONLY;
					break;
		#ifdef	O_BINARY
				case 'b':
					oflags |= O_BINARY;
					break;
		#endif
				case 'a':
					rwflags = O_WRONLY;
					oflags |= O_CREAT;
					append = 1;
					break;

				case 'w':
					rwflags = O_WRONLY;
					oflags |= O_CREAT | O_TRUNC;
					break;

				case '+':
					rwflags = O_RDWR;
					break;

	#ifdef	O_TEXT
				case 't':
					oflags |= O_TEXT;
					break;
	#endif
			}
		}

		fd = genos_open (filename, rwflags|oflags, (unsigned int)(0666));
		if(fd==-1)
			return NULL;

		if(append)
		{
#if defined (WIN32)
			if(_lseek(fd,0,SEEK_END) == -1)
			{
				_close (fd);

				return NULL;
			}
#else
			if(lseek(fd,0,SEEK_END) == -1)
			{
				close (fd);

				return NULL;
			}
#endif
		}
		else
		{
#if defined (WIN32)
			_lseek(fd,0,SEEK_SET);
#else
			lseek(fd,0,SEEK_SET);
#endif
		}

#if defined (WIN32)
		stream = _fdopen(fd,mode);
		if(stream == NULL)
		{
			_close (fd);
		}
#else
		stream = fdopen(fd,mode);
		if(stream == NULL)
		{
			close (fd);
		}
#endif
	}

	return stream;
}

int genos_fread(void * ptr, size_t size, FILE *f)
{

#if defined (DEBUG)
	printf("genos_fread : ptr=%p size=%zu file:%p\n",ptr,size,f);
#endif

	if( fread(ptr,size,1,f) != 1 )
	{
		return 1; // Error
	}
	else
	{
		return 0; // No Error
	}
}

char * genos_fgets(char * str, int num, FILE *f)
{
	return fgets( str, num, f );
}

int	genos_fclose(FILE * f)
{
#if defined (DEBUG)
	printf("genos_fclose : file:%p\n",f);
#endif

	return fclose(f);
}

int genos_statex( const char *filename, struct stat *buf)
{
#if defined (DEBUG)
	printf("genos_statex : filename=%s\n",filename);
#endif
#if defined (WIN32)
	wchar_t wpath[MAX_PATH+1];

	if(	convertpath(filename, wpath) < 0 )
		return -1;

	return _wstat (wpath,(struct _stat *)buf);
#else
	int res;
	const char *local_name;

	local_name = filename;//ToLocale( filename );

	if( local_name != NULL )
	{
		res = lstat( local_name, buf );
		// LocaleFree( local_name );
		return res;
	}

	errno = ENOENT;

	return -1;
#endif

}

int genos_stat( const char *filename, struct stat *buf)
{
 return genos_statex(filename,buf);
}

void * genos_find_first_file(char *folder, char *file, filefoundinfo* fileinfo)
{
#if defined (DEBUG)
	printf("genos_find_first_file : folder=%s file=%s\n",folder,file);
#endif

#if defined (WIN32)

	HANDLE hfindfile;
	char *folderstr;
	WIN32_FIND_DATAW FindFileData;
	wchar_t	wpath[MAX_PATH+1];

	if(file)
	{
		folderstr=(char *) malloc(strlen(folder)+strlen(file)+2);
		sprintf((char *)folderstr,"%s\\%s",folder,file);
	}
	else
	{
		folderstr = (char *) malloc(strlen(folder)+1);
		sprintf((char *)folderstr,"%s",folder);
	}

	convertpath (folderstr, wpath);
	hfindfile = FindFirstFileW(wpath, &FindFileData);
	if(hfindfile!=INVALID_HANDLE_VALUE)
	{
		WideCharToMultiByte(CP_UTF8,0,FindFileData.cFileName,-1,fileinfo->filename,sizeof(fileinfo->filename),NULL,NULL);
		//sprintf(fileinfo->filename,"%s",FindFileData.cFileName);

		fileinfo->isdirectory = 0;

		if(FindFileData.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)
		{
			fileinfo->isdirectory = 1;
		}

		fileinfo->size = FindFileData.nFileSizeLow;
		free(folderstr);
		return (void*)hfindfile;
	}
	else
	{
		free(folderstr);
		return 0;
	}

#else
	struct dirent *d;
	DIR * dir;
	struct stat fileStat;
	char * tmpstr;

	if(!file)
	{
		// File mode
		memset(&fileStat,0,sizeof(struct stat));
		if(!lstat (folder, &fileStat))
		{
			if ( !S_ISDIR ( fileStat.st_mode ) )
			{
				fileinfo->isdirectory = 0;
				fileinfo->size = fileStat.st_size;
				strncpy(fileinfo->filename,genos_getfilenamebase(folder,NULL,SYS_PATH_TYPE),FILEFOUND_NAMESIZE - 1);
				fileinfo->filename[FILEFOUND_NAMESIZE - 1] = 0;

				return (void*)-1;
			}
		}
	}

	dir = opendir (folder);
	if(dir)
	{
		d = readdir (dir);
		if(d)
		{
			tmpstr = malloc (strlen(folder) + strlen(d->d_name) + 4 );
			if(tmpstr)
			{
				strcpy(tmpstr,folder);
				strcat(tmpstr,"/");
				strcat(tmpstr,d->d_name);

				memset(&fileStat,0,sizeof(struct stat));
				if(!lstat (tmpstr, &fileStat))
				{

					if ( S_ISDIR ( fileStat.st_mode ) )
						fileinfo->isdirectory=1;
					else
						fileinfo->isdirectory=0;

					fileinfo->size=fileStat.st_size;

					strncpy(fileinfo->filename,d->d_name,FILEFOUND_NAMESIZE - 1);
					fileinfo->filename[FILEFOUND_NAMESIZE - 1] = '\0';

					free(tmpstr);
					return (void*)dir;
				}

				free(tmpstr);
			}

			closedir (dir);
			dir=0;

			return (void*)dir;
		}

		closedir (dir);
		dir=0;
	}

	return (void*)dir;

#endif

	return 0;
}

int genos_find_next_file(void* handleff, char *folder, char *file, filefoundinfo* fileinfo)
{
	int ret;

#if defined (DEBUG)
	printf("genos_find_next_file : handleff:%p folder=%s file=%s\n",handleff,folder,file);
#endif

#if defined (WIN32)
	WIN32_FIND_DATAW FindFileData;


	ret=FindNextFileW((HANDLE)handleff,&FindFileData);
	if(ret)
	{
		WideCharToMultiByte(CP_UTF8,0,FindFileData.cFileName,-1,fileinfo->filename,sizeof(fileinfo->filename),NULL,NULL);
		//sprintf(fileinfo->filename,"%s",FindFileData.cFileName);

		fileinfo->isdirectory=0;

		if( FindFileData.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY )
		{
			fileinfo->isdirectory = 1;
		}

		fileinfo->size = FindFileData.nFileSizeLow;
	}
#else
	struct dirent *d;
	DIR * dir;
	struct stat fileStat;
	char * tmpstr;

	if((long)(handleff) == -1) // File mode
		return 0;

	dir = (DIR*) handleff;
	d = readdir (dir);

	ret = 0;
	if(d)
	{
		tmpstr = malloc (strlen(folder) + strlen(d->d_name) + 4 );
		if(tmpstr)
		{
			strcpy(tmpstr,folder);
			strcat(tmpstr,"/");
			strcat(tmpstr,d->d_name);

			if(!lstat (tmpstr, &fileStat))
			{
				if ( S_ISDIR ( fileStat.st_mode ) )
					fileinfo->isdirectory=1;
				else
					fileinfo->isdirectory=0;

				fileinfo->size=fileStat.st_size;
				strncpy(fileinfo->filename,d->d_name,FILEFOUND_NAMESIZE - 1);
				fileinfo->filename[FILEFOUND_NAMESIZE - 1] = '\0';

				ret = 1;
				free(tmpstr);
				return ret;
			}

			free(tmpstr);
		}
	}

#endif

	return ret;
}

int genos_find_close(void* handle)
{
#if defined (DEBUG)
	printf("genos_find_close : handle:%p\n",handle);
#endif

#if defined (WIN32)
	if(handle)
		FindClose((void*)handle);
#else
	if((long)(handle) == -1) // File mode
		return 0;

	if(handle)
		closedir((DIR*) handle);
#endif
	return 0;
}

char * genos_getcurrentdirectory(char *currentdirectory,int buffersize)
{
	memset(currentdirectory,0,buffersize);
#if defined (WIN32)
	if(GetModuleFileName(GetModuleHandle(NULL),currentdirectory,buffersize))
	{
		if(strrchr(currentdirectory,'\\'))
		{
			*((char*)strrchr(currentdirectory,'\\')) = 0;
			return currentdirectory;
		}
	}
#else

	#if defined (OSX)

	if (_NSGetExecutablePath(currentdirectory, &buffersize) == 0)
	{
		if(strrchr(currentdirectory,'/'))
		{
			*((char*)strrchr(currentdirectory,'/')) = 0;
			return currentdirectory;
		}
	}

	#else

	strncpy(currentdirectory,"./",buffersize - 1);
	currentdirectory[buffersize-1] = '\0';

	return currentdirectory;

	#endif

#endif

	return 0;
}

int genos_mkdir(char * folder)
{
#if defined (DEBUG)
	printf("genos_mkdir : folder:%s\n",folder);
#endif

#ifdef WIN32
	_mkdir(folder);
#else
	mkdir(folder,0777);
#endif
	return 0;
}

int genos_fgetsize( FILE * f )
{
	int cur_pos,filesize;

	filesize = 0;

	if( f )
	{
		cur_pos = ftell(f);
		if( cur_pos >= 0 )
		{
			if(fseek (f , 0 , SEEK_END))
				return 0;

			filesize = ftell(f);

			if( filesize < 0 )
				filesize = 0;

			fseek (f , cur_pos , SEEK_SET);
		}

		return filesize;
	}

	return 0;
}
