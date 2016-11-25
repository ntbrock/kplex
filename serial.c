/* serial.c
 * This file is part of kplex
 * Copyright Keith Young 2012 - 2016
 * For copying information see the file COPYING distributed with this software
 *
 * This file contains code for serial-like interfaces. This currently
 * comprises:
 *     nmea 0183 serial interfaces
 *     pseudo ttys
 */

#include "kplex.h"
#include <sys/stat.h>
#include <fcntl.h>
#include <limits.h>
#if defined  __APPLE__ || defined __NetBSD__ || defined __OpenBSD__
#include <util.h>
#elif defined __FreeBSD__
#include <libutil.h>
#else
#include <pty.h>
#endif
#include <grp.h>
#include <pwd.h>

#define DEFSERIALQSIZE 32

struct if_serial_shared {
    char *device;               /* Serial device */
    int baud;                   /* baud rate of device */
    int done;                   /* Should we terminate? */
    int critical;               /* flag for critical region processing */
    int fixing;                 /* flag to indicate we're re-opening device */
    time_t retry;               /* retry interval */
    pthread_mutex_t s_mutex;    /* For synchronisation of reader and writer */
    pthread_cond_t  fv;         /* For synchronisation */
};

struct if_serial {
    int fd;
    int saved;                  /* Are stored terminal settings valid? */
    char *slavename;            /* link to pty slave (if it exists) */
    struct if_serial_shared *shared;
    struct termios otermios;    /* To restore previous interface settings
                                 *  on exit */
};

/*
 * Duplicate struct if_serial
 * Args: if_serial to be duplicated
 * Returns: pointer to new if_serial
 * Should we dup, copy or re-open the fd here?
 */
void *ifdup_serial(void *ifs)
{
    struct if_serial *oldif,*newif;

    if ((newif = (struct if_serial *) malloc(sizeof(struct if_serial)))
        == (struct if_serial *) NULL)
        return(NULL);

    oldif = (struct if_serial *) ifs;

    if ((newif->fd=dup(oldif->fd)) <0) {
        free(newif);
        return(NULL);
    }

    newif->slavename=oldif->slavename;
    newif->saved=oldif->saved;
    memcpy(&newif->otermios,&oldif->otermios,sizeof(struct termios));
    newif->shared=oldif->shared;
    return((void *)newif);
}

/*
 * Cleanup interface on exit
 * Args: pointer to interface
 * Returns: Nothing
 */
void cleanup_serial(iface_t *ifa)
{
    struct if_serial *ifs = (struct if_serial *)ifa->info;

    if (ifs->shared)
        (void)  pthread_mutex_unlock(&ifs->shared->s_mutex);

    if (!ifa->pair) {
        if (ifs->shared) {
            free(ifs->shared->device);
            free(ifs->shared);
        }
        if (ifs->saved) {
            if (tcsetattr(ifs->fd,TCSAFLUSH,&ifs->otermios) < 0) {
                if (ifa->type != PTY || errno != EIO)
                    logwarn("Failed to restore serial line: %s",strerror(errno));
            }
        }
        if (ifs->slavename) {
            if (unlink(ifs->slavename) < 0)
                logerr(errno,"Failed to remove link %s",ifs->slavename);
            free(ifs->slavename);
        }
    }
    close(ifs->fd);
}

/*
 * Open terminal (serial interface or pty)
 * Args: pathname and direction (input or output)
 * Returns: descriptor for opened interface or NULL on failure
 */
int ttyopen(char *device, enum iotype direction)
{
    int dev,flags;
    struct stat sbuf;

    errno = 0;

    /* Check if device exists and is a character special device */
    if (stat(device,&sbuf) < 0) {
        return(-1);
    }

    if (!S_ISCHR(sbuf.st_mode)){
        logerr(0,"%s is not a character device",device);
        return(-1);
    }

    /* Open device (RW for now..let's ignore direction...) */
    if ((dev=open(device,
        ((direction == OUT)?O_WRONLY:(direction == IN)?O_RDONLY:O_RDWR)|O_NOCTTY|O_NONBLOCK)) < 0) {
        return(-1);
    }

    if ((flags = fcntl(dev,F_GETFL)) < 0)
        logerr(errno,"Failed to get flags for %s",device);
    else if (fcntl(dev,F_SETFL,flags & ~O_NONBLOCK) < 0)
        logerr(errno,"Failed to set %s to non-blocking",device);

    return(dev);
}

/*
 * Set up terminal attributes
 * Args: device file descriptor,pointer to structure to save old termios,
 *     control flags and a flag indicating if this is a seatalk interface
 *     All a bit clunky and should be revised
 * Returns: 0 on success, -1 otherwise
 */
int ttysetup(int dev,struct termios *otermios_p, int baud, int st)
{
    struct termios ttermios,ntermios;

    errno = 0;

    /* Get existing terminal attributes and save them */
    if (tcgetattr(dev,otermios_p) < 0) {
        logerr(errno,"failed to get terminal attributes");
        return(-2);
    }

    memcpy(&ntermios,otermios_p,sizeof(struct termios));

    /* PARMRK is set for seatalk interface as parity errors are how we
     * identify commands
     */
    ntermios.c_iflag|=(IGNBRK|INPCK);
    if (st)
        ntermios.c_iflag |= PARMRK;
    else
        ntermios.c_iflag &= ~PARMRK;

    /* disable software flow control */
    ntermios.c_cflag &= ~(IXON | IXOFF | IXANY);

    /* CS8 1 Stop bit no parity */
    ntermios.c_cflag &= ~PARENB;
    ntermios.c_cflag &= ~CSTOPB;

    ntermios.c_cflag &= ~CSIZE;
    ntermios.c_cflag |= CS8;

    /* Enable receiver (should be a problem for sender) and ignore hardware
     * flow control */
    ntermios.c_cflag |= (CLOCAL | CREAD);

    /* set baud rate */
    cfsetispeed(&ntermios,baud);
    cfsetospeed(&ntermios,baud);

    ntermios.c_cc[VMIN]=1;
    ntermios.c_cc[VTIME]=0;

    /* select raw mode */
    cfmakeraw(&ntermios);

    if (tcsetattr(dev,TCSANOW,&ntermios) < 0) {
        logerr(errno,"Failed to set up serial line!");
        return(-1);
    }

    /* Read back terminal attributes to check we set what we needed to */
    if (tcgetattr(dev,&ttermios) < 0) {
        logerr(errno,"Failed to re-read serial line attributes");
        return(-1);
    }

    if ((ttermios.c_cflag != ntermios.c_cflag) ||
        (ttermios.c_iflag != ntermios.c_iflag)) {
        logerr(0,"Failed to correctly set up serial line");
        return(-1);
    }

    return(0);
}

/*
 * Re-open serial connection
 * Args: Pointer to interface structure
 * Returns: 0 on success, -1 otherwise
 */
int reopen_serial(iface_t *ifa)
{
    struct if_serial *ifs = (struct if_serial *) ifa->info;

    DEBUG(3,"%s: Reconnecting interface",ifa->name);
    
    /* ifs->shared->s_mutex should be locked by the calling routine */

    for (;;) {
        (void) close(ifs->fd);
        mysleep(ifs->shared->retry);
        if ((ifs->fd=ttyopen(ifs->shared->device,
                (ifa->pair)?BOTH:ifa->direction)) < 0){
            if (!(errno == ENOENT || errno == ENXIO)) {
                ifs->shared->done++;
                DEBUG(3,"failed to open %s:%s", ifs->shared->device,
                        strerror(errno));
                return(-1);
            }
            DEBUG(5,"failed to open %s:%s (retrying)",
                    ifs->shared->device,strerror(errno));
            continue;
        }
        DEBUG(3,"%s delayed open of %s succeeded",ifa->name,
                ifs->shared->device);

        if (ttysetup(ifs->fd,&ifs->otermios,ifs->shared->baud,0) < 0) {
            logerr(errno,"Failed to set up serial line");
            if (!(errno == EBADF || errno == EINTR)) {
                ifs->shared->done++;
                return(-1);
            }
            DEBUG(3,"%s: Failed to set up device %s: %s (retrying open)",
                    ifa->name,ifs->shared->device,strerror(errno));
            close(ifs->fd);
            continue;
        } else {
            DEBUG(3,"%s: Re-Initialised %s",ifa->name,ifs->shared->device);
            break;
        }
    }
    return 0;
} 

/*
 * Read from a serial interface
 * Args: pointer to interface structure pointer to buffer
 * Returns: Number of bytes read, zero on error or end of file
 */
ssize_t read_serial(struct iface *ifa, char *buf)
{
    struct if_serial *ifs = (struct if_serial *) ifa->info;
    ssize_t nread;
    int done=0;

    for (nread=0;nread<=0;) {
        if (flag_test(ifa,F_PERSIST)) {
            pthread_mutex_lock(&ifs->shared->s_mutex);
            if (ifs->fd == -1)
                done++;
            else
                ifs->shared->critical++;
            pthread_mutex_unlock(&ifs->shared->s_mutex);
            if (done) {
                nread=-1;
                break;
            }
        }

        if ((nread=read(ifs->fd,buf,BUFSIZ)) <= 0) {
            if (nread) {
                    DEBUG(3,"%s: %s",ifa->name,"Read Failed");
             } else {
                    DEBUG(3,"%s: EOF",ifa->name);
            }
    
            if (!flag_test(ifa,F_PERSIST))
                break;
            pthread_mutex_lock(&ifs->shared->s_mutex);
            if (ifs->shared->fixing) {
                pthread_cond_signal(&ifs->shared->fv);
                pthread_cond_wait(&ifs->shared->fv,&ifs->shared->s_mutex);
            } else {
                if (ifs->shared->critical == 2) {
                    ifs->shared->fixing++;
                    (void) close(ifs->fd);
                    pthread_cond_wait(&ifs->shared->fv,&ifs->shared->s_mutex);
                }
                if ((nread=reopen_serial(ifa)) < 0) {
                    if (ifa->pair)
                        ((struct if_serial *)ifa->pair->info)->fd=-1;
                    logerr(errno,"failed to re-open %s",ifs->shared->device);
                }
                if (ifs->shared->fixing) {
                    ifs->shared->fixing=0;
                    pthread_cond_signal(&ifs->shared->fv);
                }
            }
            ifs->shared->critical--;
            pthread_mutex_unlock(&ifs->shared->s_mutex);
        } else if (flag_test(ifa,F_PERSIST)) {
            pthread_mutex_lock(&ifs->shared->s_mutex);
            ifs->shared->critical--;
            if (ifs->shared->fixing)
                pthread_cond_signal(&ifs->shared->fv);
            pthread_mutex_unlock(&ifs->shared->s_mutex);
        }
    }
    return nread;
}

/*
 * Write nmea sentences to serial output
 * Args: pointer to interface
 * Returns: Nothing. errno supplied to iface_thread_exit()
 */
void write_serial(struct iface *ifa)
{
    struct if_serial *ifs = (struct if_serial *) ifa->info;
    senblk_t *senblk_p;
    int fd=ifs->fd;
    int n=0,tlen=0;
    int done=0;
    int part=0;
    char *ptr;
    char *tbuf;

    if (ifa->tagflags) {
        if ((tbuf=malloc(TAGMAX)) == NULL) {
            logerr(errno,"Disabing tag output on interface id %u (%s)",
                ifa->id,(ifa->name)?ifa->name:"unlabelled");
            ifa->tagflags=0;
        }
    }

    for (;(!done);) {
        /* NULL return from next_senblk means the queue has been shut
         * down. Time to die */
        if ((senblk_p = next_senblk(ifa->q)) == NULL)
            break;

        if (senfilter(senblk_p,ifa->ofilter)) {
            senblk_free(senblk_p,ifa->q);
            continue;
        }

        if (ifa->tagflags) {
            if ((tlen = gettag(ifa,tbuf,senblk_p)) == 0) {
                logerr(errno,"Disabing tag output on interface id %u (%s)",
                    ifa->id,(ifa->name)?ifa->name:"unlabelled");
                ifa->tagflags=0;
                free(tbuf);
            }
            ptr=tbuf;
        } else {
            ++part;
        }

        if (flag_test(ifa,F_PERSIST)) {
            pthread_mutex_lock(&ifs->shared->s_mutex);
            if (ifs->fd == -1)
                done++;
            else
                ifs->shared->critical++;
            pthread_mutex_unlock(&ifs->shared->s_mutex);
            if (done) {
                senblk_free(senblk_p,ifa->q);
                break;
            }
        }

        /* now write the tag, then the sentence */
        for (;part<2;++part) {
            if (part == 1) {
                ptr=senblk_p->data;
                tlen=senblk_p->len;
            }
            while(tlen) {
                if ((n=write(fd,ptr,tlen)) < 0) {
                    DEBUG2(3,"%s id %x: write failed",ifa->name,ifa->id);
                    break;
                }
                tlen-=n;
                ptr+=n;
            }
            if (tlen) {
                break;
            }
        }

        senblk_free(senblk_p,ifa->q);
        if (tlen) {
            if (!flag_test(ifa,F_PERSIST)) {
                break;
            }
            pthread_mutex_lock(&ifs->shared->s_mutex);
            if (ifs->shared->fixing) {
                pthread_cond_signal(&ifs->shared->fv);
                pthread_cond_wait(&ifs->shared->fv,&ifs->shared->s_mutex);
            } else {
                if (ifs->shared->critical == 2) {
                    ifs->shared->fixing++;
                    (void) close(ifs->fd);
                    pthread_cond_wait(&ifs->shared->fv,&ifs->shared->s_mutex);
                }
                if (reopen_serial(ifa) <  0) {
                    if (ifa->pair)
                        ((struct if_serial *) ifa->pair->info)->fd=-1;
                    logerr(errno,"failed to reopen serial device");
                    done++;
                }
                if (ifs->shared->fixing) {
                    ifs->shared->fixing=0;
                    pthread_cond_signal(&ifs->shared->fv);
                }
            }
            ifs->shared->critical--;
            pthread_mutex_unlock(&ifs->shared->s_mutex);
            DEBUG(7,"Flushing queue interface %s",ifa->name);
            flush_queue(ifa->q);
        } else if (flag_test(ifa,F_PERSIST)) {
            pthread_mutex_lock(&ifs->shared->s_mutex);
            ifs->shared->critical--;
            if (ifs->shared->fixing)
                pthread_cond_signal(&ifs->shared->fv);
            pthread_mutex_unlock(&ifs->shared->s_mutex);
        }
    }

    if (ifa->tagflags)
        free(tbuf);

    iface_thread_exit(errno);
}

/* For persistent interfaces which couldn't connect on startup, keep trying to
 * open the device
 * Args: pointer to interface
 * Returns: Nothing. Never returns in fact: calls read or write methods
 */

void delayed_serial_open(iface_t *ifa)
{
    struct if_serial *ifs = (struct if_serial *)ifa->info;
    struct if_serial *ifp;

    pthread_mutex_lock(&ifs->shared->s_mutex);
    if (ifs->shared->done) {
        pthread_mutex_unlock(&ifs->shared->s_mutex);
        DEBUG(3,"%s exiting (pair done)",ifa->name);
        iface_thread_exit(0);
    }

    while (ifs->fd < 0) {
        mysleep(ifs->shared->retry);
        if ((ifs->fd=ttyopen(ifs->shared->device,
                (ifa->pair)?BOTH:ifa->direction)) < 0){
            if (!(errno == ENOENT || errno == ENXIO)) {
                ifs->shared->done++;
                pthread_mutex_unlock(&ifs->shared->s_mutex);
                DEBUG(3,"failed to open %s:%s exiting", ifs->shared->device,
                        strerror(errno));
                iface_thread_exit(errno);
            }
            DEBUG(5,"failed to open %s:%s (retrying)",
                    ifs->shared->device,strerror(errno));
            continue;
        }
        DEBUG(3,"%s delayed open of %s succeeded",ifa->name,
                ifs->shared->device);

        if (ttysetup(ifs->fd,&ifs->otermios,ifs->shared->baud,0) < 0) {
            logerr(errno,"Failed to set up serial line");
            if (!(errno == EBADF || errno == EINTR)) {
                ifs->shared->done++;
                pthread_mutex_unlock(&ifs->shared->s_mutex);
                iface_thread_exit(errno);
            }
            DEBUG(3,"Failed to set up device %s: %s (retrying open)",
                    ifs->shared->device,strerror(errno));
            close(ifs->fd);
            continue;
        }

        ++ifs->saved;

        if (ifa->pair) {
            ifp=(struct if_serial *) ifa->pair->info;
            ifp->fd = ifs->fd;
            ifp->saved=ifs->saved;
            memcpy(&ifp->otermios,&ifs->otermios,sizeof(struct termios));
        }
    }
    pthread_mutex_unlock(&ifs->shared->s_mutex);

    if (ifa->direction == IN) {
        do_read(ifa);
    } else {
        write_serial(ifa);
    }
}

/*
 * Initialise a serial interface for nmea 0183 data
 * Args: interface specification string and pointer to interface structure
 * Retuns: Pointer to (completed) interface structure
 */
struct iface *init_serial (struct iface *ifa)
{
    char *devname=NULL; /* Serial device to open */
    char *eptr;         /* Used in option processing (strtol()) */
    struct if_serial *ifs;
    int baud=B4800;     /* Default for NMEA 0183. AIS will need
                   explicit baud rate specification */
    int ret;
    unsigned long retry=10;    /* Retry time for persistent interfaces */
    struct kopts *opt;
    int qsize=DEFSERIALQSIZE;
    
    for(opt=ifa->options;opt;opt=opt->next) {
        if (!strcasecmp(opt->var,"filename"))
            devname=opt->val;
        else if (!strcasecmp(opt->var,"baud")) {
            if (!strcmp(opt->val,"38400"))
                baud=B38400;
            else if (!strcmp(opt->val,"9600"))
                baud=B9600;
            else if (!strcmp(opt->val,"4800"))
                baud=B4800;
            else if (!strcmp(opt->val,"19200"))
                baud=B19200;
            else if (!strcmp(opt->val,"57600"))
                baud=B57600;
            else if (!strcmp(opt->val,"115200"))
                baud=B115200;
            else {
                logerr(0,"Unsupported baud rate \'%s\'",opt->val);
                return(NULL);
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
        } else if (!strcasecmp(opt->var,"qsize")) {
            if (!(qsize=atoi(opt->val))) {
                logerr(0,"Invalid queue size specified: %s",opt->val);
                return(NULL);
            }
        } else  {
            logerr(0,"unknown interface option %s",opt->var);
            return(NULL);
        }
    }

    if (devname == NULL) {
        logerr(0,"Must specify a serial device for serial interfaces");
        return(NULL);
    }

    /* Allocate serial specific data storage */
    if ((ifs = malloc(sizeof(struct if_serial))) == NULL) {
        logerr(errno,"Could not allocate memory");
        return(NULL);
    }

    if (flag_test(ifa,F_PERSIST)) {
        if ((ifs->shared = malloc(sizeof(struct if_serial_shared))) == NULL) {
            logerr(errno,"Could not allocate memory");
            return(NULL);
        }
        if ((ifs->shared->device=strdup(devname)) == NULL) {
            logerr(errno,"Could not allocate memory");
            return(NULL);
        }
        if (pthread_mutex_init(&ifs->shared->s_mutex,NULL) != 0) {
            logerr(errno,"serial mutex initialisation failed");
            return(NULL);
        }

        if (pthread_cond_init(&ifs->shared->fv,NULL) != 0) {
            logerr(errno,"serial condition variable initialisation failed");
            return(NULL);
        }
        ifs->shared->retry=retry;
        if (ifs->shared->retry != retry) {
            logerr(0,"retry value out of range");
            return(NULL);
        }
        ifs->shared->baud=baud;
        ifs->shared->done=0;
        ifs->shared->critical=0;
        ifs->shared->fixing=0;
    } else {
        ifs->shared = NULL;
    }

    /* Open interface or die */
    if ((ifs->fd=ttyopen(devname,ifa->direction)) < 0) {
        if (flag_test(ifa,F_IPERSIST) && (errno == ENOENT || errno == ENXIO)){
            DEBUG(3,"Failed to open serial device %s for %s: %s (retrying in %ds)",
                devname,(ifa->direction==IN)?"input":
                (ifa->direction==OUT)?"output": "input/output",strerror(errno),
                retry);
        
        } else {
            logerr(errno,"Failed to open %s",devname);
            return(NULL);
        }
    } else {
        DEBUG(3,"%s: opened serial device %s for %s",ifa->name,devname,
                (ifa->direction==IN)?"input":(ifa->direction==OUT)?"output":
                "input/output");
    }

    free_options(ifa->options);

    if (ifs->fd < 0) {
        ifa->read=delayed_serial_open;
        ifa->write=delayed_serial_open;
    } else {
        /* Set up interface or die */
        if ((ret = ttysetup(ifs->fd,&ifs->otermios,baud,0)) < 0) {
            if (ret == -1) {
                if (tcsetattr(ifs->fd,TCSANOW,&ifs->otermios) < 0) {
                    logerr(errno,"Failed to reset serial line");
                }
            }
            return(NULL);
        }
        ifs->saved=1;
        ifa->read=do_read;
        ifa->write=write_serial;
    }

    ifa->readbuf=read_serial;
    ifs->slavename=NULL;
    ifa->cleanup=cleanup_serial;

    /* Allocate queue for outbound interfaces */
    if (ifa->direction != IN)
        if (init_q(ifa, qsize) < 0) {
            logerr(errno,"Could not create queue");
            cleanup_serial(ifa);
            return(NULL);
        }

    /* Link in serial specific data */
    ifa->info=(void *)ifs;

    if (ifa->direction == BOTH) {
        if ((ifa->next=ifdup(ifa)) == NULL) {
            logerr(0,"Interface duplication failed");
            cleanup_serial(ifa);
            return(NULL);
        }
        ifa->direction=OUT;
        ifa->pair->direction=IN;
    }
    return(ifa);
}

/*
 * Initialise a pty interface. For inputs, this is equivalent to init_serial
 * Args: string specifying the interface and pointer to (incomplete) interface
 * Returns: Completed interface structure
 */
struct iface *init_pty (struct iface *ifa)
{
    char *devname=NULL;
    char* baudstr="4800";
    struct if_serial *ifs;
    int baud=B4800,slavefd;
    int ret;
    unsigned long retry=10;
    struct kopts *opt;
    int qsize=DEFSERIALQSIZE;
    char *master="s";
    char *cp;
    char *eptr;         /* Used in option processing (strtol()) */
    mode_t perm = 0;
    struct passwd *owner;
    struct group *group;
    uid_t uid=-1;
    gid_t gid=-1;
    struct stat statbuf;
    char slave[PATH_MAX];

    for(opt=ifa->options;opt;opt=opt->next) {
        if (!strcasecmp(opt->var,"mode")) {
            master=opt->val;
            if(strcmp(master,"master") && strcmp(master,"slave")) {
                logerr(0,"pty mode \'%s\' unsupported: must be master or slave",master);
                return(NULL);
            }
        }
        else if (!strcasecmp(opt->var,"filename"))
            devname=opt->val;
        else if (!strcasecmp(opt->var,"owner")) {
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
        } else if (!strcasecmp(opt->var,"baud")) {
            baudstr=opt->val;
            if (!strcmp(opt->val,"38400"))
                baud=B38400;
            else if (!strcmp(opt->val,"9600"))
                baud=B9600;
            else if (!strcmp(opt->val,"4800"))
                baud=B4800;
            else if (!strcmp(opt->val,"19200"))
                baud=B19200;
            else if (!strcmp(opt->val,"57600"))
                baud=B57600;
            else if (!strcmp(opt->val,"115200"))
                baud=B115200;
            else {
                logerr(0,"Unsupported baud rate \'%s\' in interface specification '\%s\'",opt->val,devname);
                return(NULL);
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
        } else if (!strcasecmp(opt->var,"qsize")) {
            if (!(qsize=atoi(opt->val))) {
                logerr(0,"Invalid queue size specified: %s",opt->val);
                return(NULL);
            }
        } else {
            logerr(0,"Unknown interface option %s",opt->var);
            return(NULL);
        }
    }

    if ((ifs = malloc(sizeof(struct if_serial))) == NULL) {
        logerr(errno,"Could not allocate memory");
        return(NULL);
    }

    ifs->saved=0;
    ifs->slavename=NULL;

    if (flag_test(ifa,F_PERSIST)){
        if (*master != 's') {
            logerr(0,"persist mode not valid with master ptys");
            return(NULL);
        }
        if ((ifs->shared = malloc(sizeof(struct if_serial_shared))) == NULL) {
            logerr(errno,"Could not allocate memory");
            return(NULL);
        }
        if ((ifs->shared->device=strdup(devname)) == NULL) {
            logerr(errno,"Could not allocate memory");
            return(NULL);
        }
        if (pthread_mutex_init(&ifs->shared->s_mutex,NULL) != 0) {
            logerr(errno,"serial mutex initialisation failed");
            return(NULL);
        }

        if (pthread_cond_init(&ifs->shared->fv,NULL) != 0) {
            logerr(errno,"serial condition variable initialisation failed");
            return(NULL);
        }
        ifs->shared->retry=retry;
        if (ifs->shared->retry != retry) {
            logerr(0,"retry value out of range");
            return(NULL);
        }
        ifs->shared->baud=baud;
        ifs->shared->done=0;
        ifs->shared->critical=0;
        ifs->shared->fixing=0;
    } else {
        ifs->shared = NULL;
    }

    if (*master != 's') {
        if (openpty(&ifs->fd,&slavefd,slave,NULL,NULL) < 0) {
            logerr(errno,"Error opening pty");
            return(NULL);
        }
        if (gid != -1 || uid != -1) {
            if (chown(slave,uid,gid) < 0) {
                logerr(errno,"Failed to set ownership or group for slave pty");
                return(NULL);
            }
        }
        if (perm != 0) {
            if (chmod(slave,perm) < 0) {
                logerr(errno,"Failed to set permissions for slave pty");
                return(NULL);
            }
        }
        if (devname) {
        /* Device name has been specified: Create symlink to slave */
            if (lstat(devname,&statbuf) == 0) {
            /* file exists */
                if (!S_ISLNK(statbuf.st_mode)) {
            /* If it's not a symlink already, don't replace it */
                    logerr(0,"%s: File exists and is not a symbolic link",devname);
                    return(NULL);
                }
            /* It's a symlink. remove it */
                if (unlink(devname) && errno != ENOENT) {
                    logerr(errno,"Could not unlink %s",devname);
                    return(NULL);
                }
            }
        /* link the given name to our new pty */
            if (symlink(slave,devname)) {
                logerr(errno,"Could not create symbolic link %s for %s",devname,slave);
                return(NULL);
            }
            DEBUG(3,"%s: created pty link %s to %s",ifa->name,devname,slave);

            /* Save the name to unlink it on exit */
            if ((ifs->slavename=strdup(devname)) == NULL) {
                logerr(errno,"Failed to save device name. Link will not be removed on exit");
            }
        } else {
            /* No device name was given: Just print the pty name */
            loginfo("Slave pty for output at %s baud is %s",baudstr,slave);
        }
    } else {
    /* Slave mode: This is no different from a serial line */
        if (!devname) {
            logerr(0,"Must Specify a filename for slave mode pty");
            return(NULL);
        }
        if ((ifs->fd=ttyopen(devname,ifa->direction)) < 0) {
            if (flag_test(ifa,F_IPERSIST) &&
                    (errno == ENOENT || errno == ENXIO)){
                DEBUG(3,"Failed to open serial device %s for %s: %s (retrying in %ds)",
                        devname,(ifa->direction==IN)?"input":
                        (ifa->direction==OUT)?"output": "input/output",
                        strerror(errno),retry);

            } else {
                logerr(errno,"Failed to open %s",devname);
                return(NULL);
            }
        } else {
            DEBUG(3,"%s: opened pty slave %s for %s",ifa->name,devname,
                    (ifa->direction==IN)?"input":(ifa->direction==OUT)?"output":
                    "input/output");
        }
    }

    free_options(ifa->options);

    if ((ret=ttysetup(ifs->fd,&ifs->otermios,baud,0)) < 0) {
        if (ret == -1) {
            if (tcsetattr(ifs->fd,TCSANOW,&ifs->otermios) < 0) {
                logerr(errno,"Failed to reset serial line");
            }
        }
        return(NULL);
    }
    ifs->saved=1;

    if (ifs->fd < 0) {
        ifa->read=delayed_serial_open;
        ifa->write=delayed_serial_open;
    } else {
        ifa->read=do_read;
        ifa->write=write_serial;
    }

    ifa->readbuf=read_serial;
    ifa->cleanup=cleanup_serial;

    if (ifa->direction != IN)
        if (init_q(ifa, qsize) < 0) {
            logerr(errno,"Could not create queue");
            cleanup_serial(ifa);
            return(NULL);
        }

    ifa->info=(void *)ifs;
    if (ifa->direction == BOTH) {
        if ((ifa->next=ifdup(ifa)) == NULL) {
            logerr(0,"Interface duplication failed");
            cleanup_serial(ifa);
            return(NULL);
        }
        ifa->direction=OUT;
        ifa->pair->direction=IN;
    }
    return(ifa);
}
