/* fileio.c
 * This file is part of kplex
 * Copyright Keith Young 2012 - 2016
 * For copying information see the file COPYING distributed with this software
 *
 * This file contains code for i/o from files (incl stdin/stdout)
 */

#include "kplex.h"
#include <stdlib.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/uio.h>
#include <pwd.h>
#include <grp.h>

struct if_file {
    int fd;
    int isfifo;
    off_t off;
    char *filename;
    size_t qsize;
    struct timespec retry;
};

/*
 * Duplicate struct if_file
 * Args: if_file to be duplicated
 * Returns: pointer to new if_file
 */
void *ifdup_file(void *iff)
{
    struct if_file  *newif;

    if ((newif = (struct if_file *) malloc(sizeof(struct if_file)))
        == (struct if_file *) NULL)
        return(NULL);

    memset ((void *)newif,0,sizeof(struct if_file));

    /* Read/Write only supported for stdin/stdout so don't allocate fp
     * And don't bother to duplicate filename
     */
    return(newif);
}

void cleanup_file(iface_t *ifa)
{
    struct if_file *iff = (struct if_file *) ifa->info;

    if (iff->fd >= 0)
        close(iff->fd);
    if (iff->filename)
        free(iff->filename);
}

void write_file(iface_t *ifa)
{
    struct if_file *ifc = (struct if_file *) ifa->info;
    senblk_t *sptr;
    int usereturn=flag_test(ifa,F_NOCR)?0:1;
    int data=0;
    int cnt=1;
    struct iovec iov[2];

    /* ifc->fd will only be < 0 if we're opening a FIFO.
     */
    if (ifc->fd < 0) {
        if ((ifc->fd=open(ifc->filename,O_WRONLY)) < 0) {
            logerr(errno,"Failed to open FIFO %s for writing\n",ifc->filename);
            iface_thread_exit(errno);
        }
        if (init_q(ifa,ifc->qsize) < 0) {
            logerr(errno,"Could not create queue for FIFO %s",ifc->filename);
            iface_thread_exit(errno);
        }
        DEBUG(3,"%s opened FIFO %s for writing",ifa->name,ifc->filename);
    }

    if (ifa->tagflags) {
        if ((iov[0].iov_base=malloc(TAGMAX)) == NULL) {
                logerr(errno,"%s: Disabing tag output",ifa->name);
                ifa->tagflags=0;
        } else {
            cnt=2;
            data=1;
        }
    }


    for(;;)  {
        if ((sptr = next_senblk(ifa->q)) == NULL) {
            break;
        }

        if (senfilter(sptr,ifa->ofilter)) {
            senblk_free(sptr,ifa->q);
            continue;
        }

        if (!usereturn) {
            sptr->data[sptr->len-2] = '\n';
            sptr->len--;
        }

        if (ifa->tagflags)
            if ((iov[0].iov_len = gettag(ifa,iov[0].iov_base,sptr)) == 0) {
                logerr(errno,"%s: Disabing tag output",ifa->name);
                ifa->tagflags=0;
                cnt=1;
                data=0;
                free(iov[0].iov_base);
            }

        iov[data].iov_base=sptr->data;
        iov[data].iov_len=sptr->len;
        if (writev(ifc->fd,iov,cnt) <0) {
            if (!(flag_test(ifa,F_PERSIST) && errno == EPIPE) ) {
                logerr(errno,"%s: write failed",ifa->name);
                break;
            }

            if ((ifc->fd=open(ifc->filename,O_WRONLY)) < 0) {
                logerr(errno,"%s: failed to re-open %s",ifa->name,
                        ifc->filename);
                break;
            }
            DEBUG(4,"%s: reconnected to FIFO %s",ifa->name,ifc->filename);
        }
        senblk_free(sptr,ifa->q);
    }

    if (cnt == 2)
        free(iov[0].iov_base);

    iface_thread_exit(errno);
}

void file_read_wrapper(iface_t *ifa)
{
    struct if_file *ifc = (struct if_file *) ifa->info;

    /* Create FILE stream here to allow for non-blocking opening FIFOs */
    if (ifc->fd == -1) {
        if ((ifc->fd = open(ifc->filename,O_RDONLY)) < 0) {
            if (!(flag_test(ifa,F_IPERSIST)) && errno == ENOENT) {
                logerr(errno,"Failed to open %s for reading\n",ifc->filename);
                iface_thread_exit(errno);
            }
        } else {
            DEBUG(3,"%s: opened %s for reading",ifa->name,ifc->filename);
        }
        do {
            nanosleep(&ifc->retry,NULL);
            if ((ifc->fd = open(ifc->filename,O_RDONLY)) < 0) {
                if (errno != ENOENT) {
                    logerr(errno,"Failed to open %s for reading\n",
                            ifc->filename);
                    iface_thread_exit(errno);
                }
            }
        } while (ifc->fd == -1);
    }
    do_read(ifa);
}

ssize_t read_file(iface_t *ifa, char *buf)
{
    struct if_file *ifc = (struct if_file *) ifa->info;
    struct stat sb;
    ssize_t nread;
    int ret;

    for(;;) {
        if ((nread=read(ifc->fd,buf,BUFSIZ)) <=0) {
            if (!flag_test(ifa,F_PERSIST))
                break;
            close(ifc->fd);
            if (ifc->isfifo) {
                if ((ifc->fd=open(ifc->filename,O_RDONLY)) < 0) {
                    logerr(errno,"Failed to re-open FIFO %s for reading\n",
                             ifc->filename);
                    break;
                }
                DEBUG(4,"%s: re-opened %s for reading",ifa->name,ifc->filename);
                continue;
            }
            /* Regular file in tail mode. Check every <retry> ms whether
             * file has grown and re-open if it has */
            do {
                (void) nanosleep(&ifc->retry,NULL);
                if ((ret = stat(ifc->filename,&sb)) < 0) {
                    logerr(errno,"Failed to stat %s",ifc->filename);
                    break;
                }
            } while(sb.st_size == ifc->off); 

            if (ret < 0)
                break;

            if (sb.st_size < ifc->off) {
                logerr(0,"%s shrunk! (exiting)",ifc->filename);
                break;
            }
            if ((ifc->fd=open(ifc->filename,O_RDONLY)) < 0) {
                logerr(errno,"Failed to re-open %s for reading",ifc->filename);
                break;
            }
            if (lseek(ifc->fd,ifc->off,SEEK_SET) < 0) {
                printf("off is %lld\n",ifc->off);
                logerr(errno,"Lost position in %s for reading",ifc->filename);
                close(ifc->fd);
                break;
            }
            continue;       
        } else
            break;
    }
    ifc->off+=nread;
    return nread;
}

iface_t *init_file (iface_t *ifa)
{
    struct if_file *ifc;
    struct kopts *opt;
    struct stat statbuf;
    unsigned long retry = 1;
    int ret;
    int append=0;
    uid_t uid=-1;
    gid_t gid=-1;
    struct passwd *owner;
    struct group *group;
    mode_t tperm,perm=0;
    char *cp;
    char *eptr=NULL;

    if ((ifc = (struct if_file *)malloc(sizeof(struct if_file))) == NULL) {
        logerr(errno,"Could not allocate memory");
        return(NULL);
    }

    memset ((void *)ifc,0,sizeof(struct if_file));

    ifc->qsize=DEFQSIZE;
    ifc->fd=-1;
    ifa->info = (void *) ifc;
    ifc->off=0;
    ifc->isfifo=0;

    for(opt=ifa->options;opt;opt=opt->next) {
        if (!strcasecmp(opt->var,"filename")) {
            if (strcmp(opt->val,"-"))
                if ((ifc->filename=strdup(opt->val)) == NULL) {
                    logerr(errno,"Failed to duplicate argument string");
                    return(NULL);
                }
        } else if (!strcasecmp(opt->var,"qsize")) {
            if (!(ifc->qsize=atoi(opt->val))) {
                logerr(0,"Invalid queue size specified: %s",opt->val);
                return(NULL);
            }
        } else if (!strcasecmp(opt->var,"append")) {
            if (!strcasecmp(opt->val,"yes")) {
                append++;
            } else if (!strcasecmp(opt->val,"no")) {
                append = 0;
            } else {
                logerr(0,"Invalid option \"append=%s\"",opt->val);
                return(NULL);
            }
        } else if (!strcasecmp(opt->var,"owner")) {
            if ((owner=getpwnam(opt->val)) == NULL) {
                logerr(0,"No such user '%s'",opt->val);
                return(NULL);
            }
            uid=owner->pw_uid;
        } else if (!strcasecmp(opt->var,"group")) {
            if ((group=getgrnam(opt->val)) == NULL) {
                logerr(0,"No such group '%s'",opt->val);
                return(NULL);
            }
            gid=group->gr_gid;
        }
        else if (!strcasecmp(opt->var,"perm")) {
            for (cp=opt->val;*cp;cp++) {
                if (*cp >= '0' && *cp < '8') {
                    perm <<=3;
                    perm += (*cp-'0');
                } else {
                    perm = 0;
                    break;
                }
            }
            perm &= ACCESSPERMS;
            if (perm == 0) {
                logerr(0,"Invalid permissions for tty device \'%s\'",opt->val);
                return 0;
            }
        } else if (!strcasecmp(opt->var,"retry")) {
            if (!flag_test(ifa,F_PERSIST)) {
                logerr(0,"retry valid only valid with persist option");
                return(NULL);
            }
            errno=0;
            if ((retry=strtol(opt->val,&eptr,0)) == 0 || (errno)) {
                logerr(0,"retry value %s out of range",opt->val);
                return(NULL);
            }
            if (*eptr != '\0') {
                logerr(0,"Invalid retry value %s",opt->val);
                return(NULL);
            }
        } else {
            logerr(0,"Unknown interface option %s\n",opt->var);
            return(NULL);
        }
    }

    ifc->retry.tv_sec=retry/1000;
    ifc->retry.tv_nsec=(retry%1000)*1000000;

    /* We do allow use of stdin and stdout, but not if they're connected to
     * a terminal. This allows re-direction in background mode
     */
    if (ifc->filename == NULL) {
        if (flag_test(ifa,F_PERSIST)) {
            logerr(0,"Can't use persist mode with stdin/stdout");
            return(NULL);
        }

        if (((ifa->direction != IN) &&
                (((struct if_engine *)ifa->lists->engine->info)->flags &
                K_NOSTDOUT)) ||
                ((ifa->direction != OUT) &&
                (((struct if_engine *)ifa->lists->engine->info)->flags &
                K_NOSTDIN))) {
            logerr(0,"Can't use terminal stdin/stdout in background mode");
            return(NULL);
        }
        if (ifa->direction == IN) {
            ifc->fd = STDIN_FILENO;
            DEBUG(3,"%s: using stdin",ifa->name);
        } else {
            ifc->fd = STDOUT_FILENO;
            DEBUG(3,"%s: using %s",ifa->name,
                    (ifa->direction==OUT)?"stdout":"stdin/stdout");
        }
    } else {
        if (ifa->direction == BOTH) {
            logerr(0,"Bi-directional file I/O only supported for stdin/stdout");
            return(NULL);
        }

        if ((ret=stat(ifc->filename,&statbuf)) < 0) {
            if (ifa->direction != OUT && (!flag_test(ifa,F_IPERSIST))) {
                logerr(errno,"stat %s",ifc->filename);
                return(NULL);
            }
        }
        if ((ret == 0) && S_ISFIFO(statbuf.st_mode)) {
            /* Special rules for FIFOs. Opening here would hang for a reading
             * interface with no writer. Given that we're single threaded here,
             * that would be bad
             */
            if (access(ifc->filename,(ifa->direction==IN)?R_OK:W_OK) != 0) {
                logerr(errno,"Could not access %s",ifc->filename);
                return(NULL);
            }
            ++ifc->isfifo;
        } else {
            if (flag_test(ifa,F_PERSIST) && ifa->direction==OUT) {
                logerr(0,"Can't use persist mode on %s: Output and Not a FIFO",
                        ifc->filename);
                return(NULL);
            }
            if (perm)
                tperm=umask(0);

            errno=0;
            /* If file is for output and doesn't currently exist...*/
            if (ifa->direction != IN && (ifc->fd=open(ifc->filename,
                        O_WRONLY|O_CREAT|O_EXCL|((append)?O_APPEND:0),
                        (perm)?perm:0664)) >= 0) {
                if (gid != 0 || uid != -1) {
                    if (chown(ifc->filename,uid,gid) < 0) {
                        logerr(errno, "Failed to set ownership or group on output file %s",ifc->filename);
                        return(NULL);
                    }
                }
            DEBUG(3,"%s: created %s for output",ifa->name,ifc->filename);
            } else {
                if (errno && errno != EEXIST) {
                    logerr(errno,"Failed to create file %s",ifc->filename);
                    return(NULL);
                }
                /* file is for input or already exists */
                if ((ifc->fd=open(ifc->filename,(ifa->direction==IN)?O_RDONLY:
                        (O_WRONLY|((append)?O_APPEND:O_TRUNC)))) < 0 &&
                        (!(errno == ENOENT && (flag_test(ifa,F_IPERSIST))))) {
                    logerr(errno,"Failed to open file %s",ifc->filename);
                    return(NULL);
                }
                DEBUG(3,"%s: opened %s for %s",ifa->name,ifc->filename,
                        (ifa->direction==IN)?"input":"output");
            }
            /* reset umask: not really necessary */
            if (perm)
                (void) umask(tperm);
        }
    }

    free_options(ifa->options);

    ifa->write=write_file;
    ifa->read=file_read_wrapper;
    ifa->readbuf=read_file;
    ifa->cleanup=cleanup_file;

    if (ifa->direction != IN && ifc->fd >= 0)
        if (init_q(ifa, ifc->qsize)< 0) {
            logerr(0,"Could not create queue");
            cleanup_file(ifa);
            return(NULL);
        }

    if (ifa->direction == BOTH) {
        if ((ifa->next=ifdup(ifa)) == NULL) {
            logerr(0,"Interface duplication failed");
            cleanup_file(ifa);
            return(NULL);
        }
        ifa->direction=OUT;
        ifa->pair->direction=IN;
        ifc = (struct if_file *) ifa->pair->info;
        ifc->fd=STDIN_FILENO;
    }
    return(ifa);
}

