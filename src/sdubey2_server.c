/* **********************************************
 *        Distance Vector Routing Algorithm     * 
 *          Author : Santosh Kumar Dubey		*
 *				Date : May-02-2014				*
 * **********************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include <unistd.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <signal.h>
#include <errno.h>
#include <time.h>
#include <netdb.h>
#include <netinet/in.h>


/* infinite cost define, the largest integer of "short int" */
#define COST_INFINITE	0X7FFF

/* define of interval */
int time_interval = 3;
/* current server listen socket */
int sockfd = -1;
/* define for commands */
int num_packets = 0;
int crash_flag = 0;		/* 1: crash		0:normal*/

/* profile for one server */
struct server {
	int id;			/* server id */
	char ip[16];	/* server ip */
	char hostname[NI_MAXHOST];	/*hostname*/
	unsigned int uip;
	int port;		/* server port */

	int isneighbor;	/* 1: is neighbor	0: not neighbor */
	int cost;		/* path cost */
	int forwardid;
	int pathclose;	/* 1: path is closed	0: path not close */

	time_t last_time;	/* last update time */
};

int num_servers = 0;	/* number servers */
int num_neighbors = 0;	/* number of neighbors */
char hostname[NI_MAXHOST];	/*host name of current server*/
int cur_serverid = 0;
struct server *cur_server = NULL;
/* info for all servers */
struct server *servers = NULL;

int *map_ids = NULL;	/* server id to server index map */
int **distance_table = NULL;	/*Distance Table*/


/*======================== define for message =======================*/
/*  -> Defined two parts for the update message, 
	   one is header of the message, and the other
	   is the field of the message.
	-> The above two structures “head_t” and “field_t” 
	   is for sending and receiving messages.
	-> When we need to send and receive messages, so 
	   firstly we need allocate a buffer, Then define 
	   two pointer ‘h’ and ‘f’, to operate the ‘head_t’ and ‘field_t’.
	   It is very convenient
======================================================================*/
typedef struct {
	short num_fields;
	short port;
	unsigned int ip;
} __attribute__((packed)) head_t;
typedef struct {
	unsigned int ip;
	short port;
	short reserved;
	short id;
	unsigned short cost;
} __attribute__((packed)) field_t;

/* find server index by server id */
int get_server_index(int serverid)
{
	int i;
	for(i = 0; i < num_servers; ++i) {
		if(serverid == servers[i].id)
			return i;
	}
	return -1;
}
struct server *get_server(int serverid)
{
	int index = get_server_index(serverid);
	if(index == -1) {
		fprintf(stderr, "BUG: invalid call, can not find server id %d\n", serverid);
		return NULL;
	}

	return &(servers[index]);
}
/* Initializing the distance table */
void init_distance_table()
{
	int i, j;

	distance_table = (int**)malloc(num_servers * sizeof(int *));
	for(i = 0; i < num_servers; ++i) {
		distance_table[i] = (int*)malloc(num_servers * sizeof(int));
		for(j = 0; j < num_servers; ++j) {
			distance_table[i][j] = COST_INFINITE;
		}
	}
}
/* Setting the distance table */
void set_distance_table(int fromid, int dstid, int cost)
{
	int fi, di;
	fi = get_server_index(fromid);
	di = get_server_index(dstid);
	if(fi == -1 || di == -1) {
		return;
	}
	/* path from x to x always zero */
	if(fi == di)
		cost = 0;
	distance_table[fi][di] = cost;
}
/* Displaying the distance table */
void display_distance_table()
{
	int i;
	printf("\n2. Distance Table for Server<%d><%s>:\n     ", cur_serverid, cur_server->hostname);
	for(i = 0; i < num_servers; ++i) {
		struct server *s = &(servers[i]);
		printf("%2d ", s->id);
	}
	printf("\n    ");
	for(i = 0; i < num_servers; ++i) {
		printf("---");
	}
	printf("\n");
	for(i = 0; i < num_servers; ++i) {
		struct server *s = &(servers[i]);
		printf("%2d | ", s->id);
		int j;
		for(j = 0; j < num_servers; ++j) {
			if(distance_table[i][j] == COST_INFINITE)
				printf(" - ");
			else
				printf("%2d ", distance_table[i][j]);
		}
		printf("\n");
	}

	printf("\n3. Cost Table for Server<%d><%s>: ", cur_serverid, cur_server->hostname);
	
	for(i = 0; i < num_servers; ++i) {
		struct server *s = &(servers[i]);
		if(s->cost == COST_INFINITE)
			printf("inf ");
		else
			printf("%d ", s->cost);
	}
	printf("\n\n====================== End of Display Command ============================");
	printf("\n\n");
}

void cmd_display();
void process_path_close(int dstid)
{
	int i, dstindex;
	struct server *s;
	dstindex = get_server_index(dstid);
	if(dstindex == -1)
		return;
	s = &(servers[dstindex]);

	s->cost = COST_INFINITE;
	s->forwardid = -1;
	for(i = 0; i < num_servers; ++i) {
		distance_table[dstindex][i] = COST_INFINITE;
	}
}

/* =========================== Function for  : Bellman-ford equation ======================= */
/* 
	*	Bellman-Ford equation: Dx(y) = minv{c(x,v) + Dv(y)}
	*   input: y (index of destination server, not server id 
	*	If the destination ‘id’ is just current server, set the cost to zero and return;
	*	Set the minimum cost to the direct cost from current server to the destination server;
	*	Travel through all servers, which are neighbors and the path is not closed, calculate 
		the cost from that server, if it is less than the minimum cost, reset the minimum cost
		and set the next hop server to that server;
	*	Update the distance table and forward id.
=============================================================================================*/
void Bellman_Ford_Equation(int y)
{
	int v, x;
	int d, min, forward;
	struct server *sy;

	/* init start server index, cost and forward */
	x = get_server_index(cur_serverid);
	/* if source and destination server are the same, just return */
	if(x == y) {
		cur_server->cost = 0;
		cur_server->forwardid = cur_serverid;
		distance_table[x][x] = 0;
		return;
	}

	/* set the init distance to the cost directly from x to y */
	sy = &(servers[y]);
	min = sy->cost;
	forward = y;

	/* calculate all neighbors of current server */
	for(v = 0; v < num_servers; ++v) {
		struct server* sv = &(servers[v]);
		/* if is not neighbor or path closed, go to next server */
		if(!sv->isneighbor)
			continue;
		if(sv->pathclose)
			continue;
		/* calculate the min path */
		d = sv->cost + distance_table[v][y];
		if(d < min) {
			min = d;
			forward = map_ids[v];
		}
	}
/* update distance table. */
	distance_table[x][y] = min;
	servers[y].forwardid = forward;
}
/*============================= function for : Updating Distance Vector ==================*/
/* In the DV algorithm, a node x updates its distance-vector estimate when
	  1. it either sees a cost change in one of its directly attached links 
	  2. or receives a distance-vector update from some neighbor.
	  3. Travel through all servers, call function Bellman_Ford_Equation to 
	     calculate the distance for each server.
==========================================================================================*/
void update_distance_vector()
{
	int y;

	/* 	13 for each y in N:
		14 Dx(y) = minv{c(x,v) + Dv(y)} */
	for(y = 0; y < num_servers; ++y) {
		Bellman_Ford_Equation(y);
	}

	/* output DEBUG log after every time we recalculate the table */
	//display_distance_table();
}

void sort_servers()
{
	int i, j, min;
	/* sort by server id */
	for(i = 0; i < num_servers - 1; ++i) {
		min = i;
		for(j = i + 1; j < num_servers; ++j) {
			if(servers[min].id > servers[j].id)
				min = j;
		}
		if(min != i) {
			struct server tmp = servers[min];
			servers[min] = servers[i];
			servers[i] = tmp;
		}
	}

	/* init server id to server index map */
	map_ids = (int*)malloc(num_servers * sizeof(int));
	for(i = 0; i < num_servers; ++i) {
		map_ids[i] = servers[i].id;
	}
}
/* Initializing the servers */
void init_server(struct server *s) {
	s->id = 0;
	bzero(s->ip, sizeof(s->ip));
	s->port = 0;
	s->cost = COST_INFINITE;
	s->isneighbor = 0;
	s->forwardid = -1;
	s->pathclose = 0;
	s->last_time = time(NULL);
	bzero(s->hostname, NI_MAXHOST);
}
/* This function is for closing socket when user pressed ctrl+C */
void free_servers()
{
	int i;
	if(sockfd >= 0) {
		close(sockfd);
		sockfd = -1;
		printf("DEBUG: free server socket\n");
		printf("\nGood bye!!!\n\n");
	}
	if(distance_table) {
		for(i = 0; i < num_servers; ++i) {
			free(distance_table[i]);
		}
		free(distance_table);
		distance_table = NULL;
	}
	if(map_ids) {
		free(map_ids);
		map_ids = NULL;
	}
	if(servers) {
		free(servers);
		servers = NULL;
	}
}
/* Function to handle SIGINT (Ctrl+C).
 * This function exits cleanly on receiving SIGINT (Ctrl+C) from the user */
void exit_handler(int sig)
{
	if(sig == SIGINT) {
		printf("\nDEBUG: got signal SIGINT\n");
		free_servers();
		exit(0);
	}
	return;
}

/*
 * Function to find out the IP address on which the process should listen
 * The IP Address should be the Public IP address of the system
 * This is found by connecting to a public DND server (8.8.8.8)
 * and reading the IP address of the socket
 *
 * Returns the sockaddr_in structure representing the address
 */
struct sockaddr_in getmyip()
{
    int sockfd;
    struct sockaddr_in servaddr, myaddr;
    socklen_t len;

    len = sizeof(struct sockaddr_in);
    sockfd=socket(AF_INET,SOCK_DGRAM,0);
	/* Checking if the creation is done successfully. */
    if(sockfd < 0) {
        printf("\nError is socket()\n");
        exit(1);
    }

    /* Connect to a Public DNS server */
    bzero(&servaddr,sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr=inet_addr("8.8.8.8");
    servaddr.sin_port=htons(53);
	/* Checking if it is connecting */
    if (connect(sockfd, (struct sockaddr *)&servaddr, sizeof(struct sockaddr_in)) < 0) {
        printf("\nError in connect()\n");
        exit(1);
    }
	/* Checking if the error is in socketname */
    if (getsockname(sockfd,(struct sockaddr *)&myaddr,&len) < 0) {
        printf("\nError in getsockname()\n");
        exit(1);
    }

	/* Get the hostname of the server */
	getnameinfo((struct sockaddr *) &myaddr, sizeof(struct sockaddr_in), hostname, NI_MAXHOST, NULL, 0, 0);

    return myaddr;
}
void get_hostname(struct sockaddr* addr, struct server *s)
{
	/* Get the hostname */
	getnameinfo(addr, sizeof(struct sockaddr_in), s->hostname, NI_MAXHOST, NULL, 0, 0);
	printf("Retrieved Hostname<%s> for server<%d> from received message.\n", s->hostname, s->id);
}
/* Find the server-ID for current server */
int set_current_serverid()
{
	int i;
	struct sockaddr_in myaddr = getmyip();
	printf("\n=====================================================");
	printf("\nMy Machine/Server Details :\n");
	printf("~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
	printf("My ip=<%s>\nMy hostname=<%s>\n", inet_ntoa(myaddr.sin_addr), hostname);
	/* search all servers */
	for(i = 0; i < num_servers; ++i) {
		struct server *s = &(servers[i]);
		in_addr_t addr = inet_addr(s->ip);
		/* check the ip is the same or not */
		if(addr == myaddr.sin_addr.s_addr) {
			/* set current server id */
			cur_serverid = s->id;
			cur_server = s;
			strcpy(s->hostname, hostname);
			printf("My server id=%d\n", cur_serverid);
			return 0;
		}
	}
	return -1;
}
/*====function for : loading and reading config-file from directory and processing them ===== */
/*    
*	Open the config file (Topology file);
*	Read the first line number of servers;
*	Read the second line number of neighbors;
*	Allocate memory and init them for all servers;
*	Read in server id, IP, port for each server;
*	Sort all servers by server-id from small to big;
*	Find out the server id of current server, by comparing the IP address;
*	Init distance table;
*	Read all neighbors of the server;
*	Close the config file.
================================================================================================*/
int load_config(char *filename)
{
	int i;
	/* open config file */
	FILE *fp = fopen(filename, "r");
	if(fp == NULL) {
		fprintf(stderr, "open config file %s failed\n", filename);
		return -1;
	}

	/* read in number of servers and neighbors */
	if(fscanf(fp, "%d", &num_servers) != 1) {
		fprintf(stderr, "First line should be number of server\n");
		fclose(fp);
		return -1;
	}
	/* Checking if the first line is neighbors */
	if(fscanf(fp, "%d", &num_neighbors) != 1) {
		fprintf(stderr, "First line should be number of neighbors\n");
		fclose(fp);
		return -1;
	}

	/* allocate memory for all servers */
	servers = (struct server*)malloc(num_servers * sizeof(struct server));
	for(i = 0; i < num_servers; ++i) {
		struct server *s = &(servers[i]);
		/* init each server */
		init_server(s);
		/* read in ip and port of each server */
		if(fscanf(fp, "%d%s%d", &(s->id), s->ip, &(s->port)) != 3) {
			fprintf(stderr, "Config for server format error\n");
			fclose(fp);
			return -1;
		}
		s->uip = inet_addr(s->ip);
	}
	/* sort all server by server id, from small to big */
	sort_servers();

	/* find current server id */
	if(set_current_serverid() != 0) {
		fprintf(stderr, "can not find config for current server\n");
		fclose(fp);
		return -1;
	}

	/* init distance table */
	init_distance_table();

	/* read in all neighbors */
	for(i = 0; i < num_neighbors; ++i) {
		int fromid, dstid, cost;
		struct server *s;
		if(fscanf(fp, "%d%d%d", &fromid, &dstid, &cost) != 3) {
			fprintf(stderr, "Config for neighbors format error\n");
			fclose(fp);
			return -1;
		}
		/* Checking if server-id is correct in the column */
		if(fromid != cur_serverid && dstid != cur_serverid) {
			fprintf(stderr, "Error: First column or second column must be current server id<%d>.config [%d %d %d]\n",
				cur_serverid, fromid, dstid, cost);
			fclose(fp);
			return -1;
		}
		if(fromid != cur_serverid) {
			dstid = fromid;
			fromid = cur_serverid;
		}
		s = get_server(dstid);
		/* Checking if the destination server-id is correct */
		if(s == NULL) {
			fprintf(stderr, "Error: Destination server id<%d> not exist. config [%d %d %d]\n",
				dstid, fromid, dstid, cost);
			fclose(fp);
			return -1;
		}
		/* Checking if the cost is bigger than zero */
		if(cost <= 0) {
			fprintf(stderr, "Error: cost must be bigger than zero. config [%d %d %d]\n",
				fromid, dstid, cost);
			fclose(fp);
			return -1;
		}
		/* set cost and Dx(y) for all neighbors */
		s->cost = cost;
		s->forwardid = dstid;
		s->isneighbor = 1;
		set_distance_table(cur_serverid, dstid, cost);
	}
	/* set cost and Dx(y) for current server */
	cur_server->cost = 0;
	cur_server->forwardid = cur_serverid;
	cur_server->isneighbor = 0;
	set_distance_table(cur_serverid, cur_serverid, 0);
	printf("\nDEBUG: reading from config-file successfully done!!\n");
	printf("=====================================================\n");
	fclose(fp);
	return 0;
}

int send_update_message();
/* For "display" command */
void cmd_display()
{
	int i;
	int cur_index = get_server_index(cur_serverid);
	printf("\n======================= Inside Display Command ==========================\n");
	printf("\n1. Table for Server<%d><%s>\n", cur_serverid, hostname);
	printf("\n==============================\n");
	printf("D.server-id | next-hop | cost\n");
	printf("==============================\n");
	for(i = 0; i < num_servers; ++i) {
		struct server *s = &(servers[i]);
		if(distance_table[cur_index][i] == COST_INFINITE) {
			printf("      %d \t  %s \t   %s\n", s->id, "-", "inf");
		} else {
			printf("      %d \t  %d \t   %d\n", s->id, s->forwardid, distance_table[cur_index][i]);
		}
	}

	// output Distance Table and cost Table
	display_distance_table();
}
/* For "step" command */
void cmd_step()
{
	send_update_message();
}
/* For "packets" command */
void cmd_packets()
{
	printf("The number of packets is: %d\n", num_packets);
	num_packets = 0;
}
/* For "crash" command */
void cmd_crash()
{
	int i;
	/* close all path and set route table to infinite */
	for(i = 0; i < num_servers; ++i) {
		struct server *s = &(servers[i]);
		s->pathclose = 1;
		s->cost = COST_INFINITE;
		s->forwardid = -1;
	}

	/* set the server to crashed */
	crash_flag = 1;
}
/*===================Function for : "disable" command =====================*/
/*
*	Read the first parameter, which is the destination server id;
*	If the destination server id does not exist, return an error;
*	If the destination server is not a neighbor, return an error;
*	Set the path to closed;
*	Recalculate the distance vector;
==========================================================================*/
int cmd_disable(char *cmd)
{
	int serverid = -1;
	struct server *s;
	if(strlen(cmd) > 8) {
		serverid = atoi(cmd+8);
	}
	s = get_server(serverid);
	/* Checking if the client forgotten to enter server-ID */
	if(s == NULL || !s->isneighbor) {
		printf("Please input the neighbor server-ID\n");
		return -1;
	}
	/* Checking if the path is disabled */
	if(s->pathclose) {
		printf("Warning: path to Server<%d> is already disabled or lost connection\n", serverid);
	}
	s->pathclose = 1;
	process_path_close(serverid);
	update_distance_vector();
	return 0;
}
/*======================Function for : "update" command =============================*/
/*
*	Read the three parameters id1 id2 and cost;
*	If the input cost is a string "inf", set the cost to an integer COST_INFINITE;
*	The input cost must between 0 to COST_INFINITE
*	If server id1 or id2 does not exist, return an error;
*	If id1 equals to id2, return an error;
*	We must make sure there is only one ‘id’ is the ‘id’ of current server;
*	If the destination server is not a neighbor, return an error;
*	If the path already closed, return an error;
*	Update the cost;
*	Recalculate the distance vector.
=====================================================================================*/
int cmd_update(char *cmd)
{
	int id1, id2, icost;
	char cost[16];
	struct server *s2;

	/* check length of command */
	if(strlen(cmd) < 8) {
		printf("Usage: update id1 id2 cost\n");
		return -1;
	}
	/* read in id1, id2 and cost params */
	bzero(cost, sizeof(cost));
	if(sscanf(cmd+6, "%d%d%s", &id1, &id2, cost) != 3) {
		printf("Please input the id1 id2 cost\n");
		return -1;
	}
	if(strncmp(cost, "inf", 3) == 0) {
		/* we only allow "inf", "inff" is not correct */
		if(strlen(cost) != 3) {
			printf("Cost for infinite must be \"inf\", not others\n");
			return -1;
		}
		icost = COST_INFINITE;
	} else {
		icost = atoi(cost);
	}
	/* check the range of cost, it must be between 0 and COST_INFINITE */
	if(icost < 0 || icost > COST_INFINITE) {
		printf("Cost must between 0 to %d\n", COST_INFINITE);
		return -1;
	}

	/* check if it is current server */
	if(id1 != cur_serverid) {
		printf("Please enter update command in server %d\n\n", id1);
		return -1;
	}
	/* check if id1 equals to id2 */
	if(id1 == id2) {
		printf("The cost from current server to current server is always 0\n");
		return -1;
	}
	/* set id1 equal current server id, id2 equal destination server id */
	if(id2 == cur_serverid) {
		id2 = id1;
		id1 = cur_serverid;
	}
	/* check id input is correct or not */
	s2 = get_server(id2);
	if(s2 == NULL) {
		printf("Server id %d do not exist\n", id2);
		return -1;
	}
	/* check is neighbor */
	if(!s2->isneighbor) {
		printf("Server id %d and Server id %d are not neighbor.\n", cur_serverid, id2);
		return -1;
	}
	/* check path is disabled or not */
	if(s2->pathclose) {
		printf("Path from %d to %d already disabled\n", cur_serverid, id2);
		return -1;
	}

	//update cost
	//set_distance_table(id1, id2, icost);
	s2->cost = icost;

	update_distance_vector();

	return 0;
}
/*Showing the command when user type*/
void output_response(char *cmd, char *errmsg)
{
	if(errmsg == NULL) {
		printf("%s SUCCESS\n", cmd);
	} else {
		printf("%s %s\n", cmd, errmsg);
	}
}
/*======== Function for : Taking the command input and process it =============*/
/*
*	Read the data from standard input;
*	If this server already crashed, do nothing;
*	Remove the last new line of input buffer;
*	Distinguish the commands, and call different functions to process them;
			*	call cmd_step for command step;
			*	call cmd_packets for command packets;
			*	call cmd_display for command display;
			*	call cmd_crash for command crash;
			*	call cmd_disable for command disable;
			*	call cmd_update for command update;
*	If the inserted commands do not match, output an error hint;
*	If the command execute successfully, output "command SUCCESS".
================================================================================*/
void read_commands(int fd)
{
	int ret = 0;
	char buffer[1024];
	int count = read(fd, buffer, sizeof(buffer) - 1);
	/* if the server crashed, we do not response any commands */
	if(crash_flag) {
		return;
	}
	/* not invalid command input */
	if(count <= 1) {
		return;
	}
	/* make the input string ended with 0 */
	buffer[count] = 0;
	/* remove the end new line */
	if(buffer[count-1] == '\n') {
		buffer[count-1] = 0;
	}
	/* Checking all the command inserted by the user and displaying them */
	printf("DEBUG: your input command is [%s]\n", buffer);

	if(strcasecmp(buffer, "step") == 0) {
		cmd_step();
	} else if(strcasecmp(buffer, "packets") == 0) {
		cmd_packets();
	} else if(strcasecmp(buffer, "display") == 0) {
		cmd_display();
	} else if(strcasecmp(buffer, "crash") == 0) {
		cmd_crash();
	} else if(strncasecmp(buffer, "disable", 7) == 0) {
		ret = cmd_disable(buffer);
		/* we only need to output "disable SUCCESS",
		   do not need to output the params */
		buffer[7] = '\0';
	} else if(strncasecmp(buffer, "update", 6) == 0) {
		ret = cmd_update(buffer);
		/* we only need to output "disable SUCCESS",
		   do not need to output the params */
		buffer[6] = '\0';
	} else {
		printf("Unknown command, please re-input it or refer MANUAL.\n");
		ret = -1;
	}

	if(!ret) {
		output_response(buffer, NULL);
		return;
	}
}

/* ========================Function for : send out current distance route to neighbors ========================*/
/*
*	Fill the IP, port, number of fields to the header;
*	Set IP, port, cost, id for all fields;
*	Travel through all server, if we are neighbor and the path is not closed, send out message to that server.
===============================================================================================================*/
int send_update_message()
{
	int num_fields = num_servers;
	int size = sizeof(head_t) + num_fields * sizeof(field_t);
	char packet[1024];
	head_t *h = (head_t*)packet;
	field_t *f = (field_t*)(packet + sizeof(head_t));
	struct server *s = cur_server;
	int cur_index = get_server_index(cur_serverid);
	int i;

	/* set header */
	h->num_fields = num_fields;
	h->port = htons(s->port);
	h->ip = s->uip;

	/* set all update fields */
	for(i = 0; i < num_fields; ++i) {
		s = &(servers[i]);
		f->ip = s->uip;
		f->port = htons(s->port);
		f->reserved = 0;
		f->id = s->id;
		f->cost = distance_table[cur_index][i];
		//printf("DEBUG: Sending distance fields: %d --> %d = %d\n", cur_serverid, f->id, f->cost);
		++f;
	}

	/* send out to all neighbors */
	for(i = 0; i < num_servers; ++i) {
		struct sockaddr_in clientaddr;
		s = &(servers[i]);
		/* Checking if the server is in the network, if it is not then do not sent route fields. */
		if(!s->isneighbor)
			continue;
		if(s->pathclose) {
			printf("DEBUG: Dropping route fields to disappeared server<%d><%s>\n", s->id, s->hostname);
			continue;
		}
		bzero(&clientaddr, sizeof(clientaddr));
		clientaddr.sin_family = AF_INET;
		clientaddr.sin_addr.s_addr = s->uip;
		clientaddr.sin_port = htons(s->port);
		/* Displaying sending route fields with the details*/
		printf("\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
		printf("\nDEBUG: Sending route fields to server<%d>\n", s->id);
		printf("Address  : <%s : %u>\nHostname : <%s>\n", s->ip, s->port, s->hostname);
		printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n\n");
		sendto(sockfd, packet, size, 0, (struct sockaddr*)&clientaddr, sizeof(clientaddr));
	}
	
	return 0;
}
/*==============Function For : receive packets from neighbors in proper message format =======================*/

/*
*	Read the packet from the listened port;
*	Check the packet size, it must be bigger than the size of struct head_t;
*	Read the fields number from header, and check the size of all fields;
*	Compare the senders IP with all neighbors, if it is not a neighbor of current server, just drop the packet;
*	Get the host name of the sender server;
*	If the path from that server is already closed(for example crash, disable), drop the packet;
*	Update the last active time of the sender to current time;
*	Read all distance from the fields;
*	Recalculate the distance vector again.
*/
void read_messages(int fd)
{
	int i, count;
	char buffer[1024];
	struct server *from_server = NULL;
	struct sockaddr_in from_addr;
	socklen_t from_len = sizeof(struct sockaddr_in);
	head_t *h;
	field_t *f;

	/* receive message from neighbor */
	count = recvfrom(fd, buffer, sizeof(buffer), 0,  (struct sockaddr*)&from_addr, &from_len);
	if(count < sizeof(head_t)) {
		fprintf(stderr, "Wrong General Message Format\n");
		return;
	}
	/* check packet size */
	h = (head_t*)buffer;
	if(count != (8 + h->num_fields * 12)) {
		fprintf(stderr, "Wrong General Message Format\n");
		return;
	}
	/* get neighbor server */
	for(i = 0; i < num_servers; ++i) {
		struct server *s = &(servers[i]); 
		if(s->uip == h->ip && s->port == ntohs(h->port)) {
			from_server = s;
			break;
		}
	}
	/* Checking if the server is still in the network after it sends the packets/message */
	if(from_server == NULL) {
		fprintf(stderr, "Cannot find server who sends the message\n");
		return;
	}

	/* only the first time we need to get the hostname,
	   sequence requests we do not need any more */
	if(strlen(from_server->hostname) == 0) {
		get_hostname((struct sockaddr*)&from_addr, from_server);
	}

	/* if the path already close, do not receive any packet */
	if(from_server->pathclose) {
		printf("DEBUG: Dropping route fields message from closed path server<%d><%s>\n", from_server->id, from_server->hostname);
		return;
	}
	/*Showing the received message*/
	printf("\n\n****************************************\n");
	printf("*  RECEIVED A MESSAGE FROM SERVER <%d>  *\n", from_server->id);
	printf("****************************************\n");
	printf("Address  : <%s : %u>\nHostname : <%s>\n", from_server->ip, from_server->port, from_server->hostname);
	printf("----------------------------------------\n\n");
	++num_packets;
	from_server->last_time = time(NULL);

	f = (field_t*)(buffer+sizeof(head_t));
	for(i = 0; i < h->num_fields; ++i) {
		//printf("DEBUG: Receiving distance fields: %d --> %d = %d\n", from_server->id, f->id, f->cost);
		set_distance_table(from_server->id, f->id, f->cost);
		++f;
	}

	update_distance_vector();
}
/*===============function for : init and start the server =====================================*/
/* 
*	create a new socket;
*	set server socket address;
*	bind the socket to the address;
*	register a signal handler for SIGINT, because we need to release the socket after exit;
*	send route information to all neighbors;
*	set an alarm clock after time_interval seconds;
*	run a forever loop try to listen input from network or user input;
===============================================================================================*/
int start_server()
{
	fd_set rfds;
	int retval;
	/* create socket */
	struct sockaddr_in servaddr;
	sockfd = socket(AF_INET, SOCK_DGRAM, 0);
	if(sockfd == -1) {
		fprintf(stderr, "create socket failed\n");
		exit(EXIT_FAILURE);
	}

	/* set address */
	bzero(&servaddr, sizeof(servaddr)); /* init all elements to zero */
	servaddr.sin_family = AF_INET;		/* set the family for IPv4 */
	servaddr.sin_addr.s_addr = htonl(INADDR_ANY); /* set the listen address to 0.0.0.0, it can accept messages from any Ethers */
	servaddr.sin_port = htons(cur_server->port);  /* set the listen port, the server will receive messages from this port */

	/* bind address and port to socket */
	if(bind(sockfd, (struct sockaddr *)&servaddr, sizeof(servaddr)) == -1)
	{
		perror("bind failed\n");
		close(sockfd);
		exit(EXIT_FAILURE);
	}

    /* Register the signal handler to handle Ctrl+C */
	if(signal(SIGINT, exit_handler) == SIG_ERR) {
		printf("\nError registering signal handler\n");
		exit(EXIT_FAILURE);
	}

	/* start timer */
	send_update_message();
	alarm(time_interval);

	FD_ZERO(&rfds);
	FD_SET(0, &rfds);
	FD_SET(sockfd, &rfds);
	/* forever loop */
	while(1) {
		fd_set tmp_rfds = rfds;
		/* read in commands and receive packets from neighbors */
		retval = select(sockfd+1, &tmp_rfds, NULL, NULL, NULL);
		if (retval == -1) {
			if(errno == EINTR)
				continue;
			perror("select");
			exit(EXIT_FAILURE);
		}
		else {
			if(FD_ISSET(0, &tmp_rfds)) {
				read_commands(0);
			}
			if(FD_ISSET(sockfd, &tmp_rfds)) {
				read_messages(sockfd);
			}
		}
	}

	return 0;
}

/* send out message every time_interval seconds */
void alarm_handler(int sig)
{
	time_t now = time(NULL);
	int i;
	int lost_flag = 0;
	/* if we do not receive update messages after 3 time interval,
	   set the cost to infinite and disable the path.
	   Then the server can never go back again. */
	for(i = 0; i < num_servers; ++i) {
		struct server *s = &(servers[i]);
		if(!s->isneighbor)
			continue;
		if(s->pathclose)
			continue;
		if(now - s->last_time > 3*time_interval) {
			lost_flag = 1;
			set_distance_table(cur_serverid, s->id, COST_INFINITE);
			/* if a node disappeared, just set the cost to infinite,
			   do not disable the path, process it as "update n1 n2 inf" */
			//s->pathclose = 1;
			//process_path_close(s->id);
			s->cost = COST_INFINITE;
			printf("DEBUG: server <%d> disappeared from network\n", s->id);
		}
	}
	if(lost_flag) {
		update_distance_vector();
	}

	/* send update message to neighbors */
	send_update_message();

	/* reset the timer again */
	alarm(time_interval);
}

int main(int argc, char **argv)
{
	int interval = 0;
	char *fn = NULL;
	int opt;

	/* read command line inputs */
	while ((opt = getopt(argc, argv, "t:i:")) != -1) {
		switch (opt) {
		case 'i':
			interval = atoi(optarg);
			break;
		case 't':
			fn = optarg;
			break;
		default: /* '?' */
			fprintf(stderr, "\nIncorrect arguments, Please refer Help\n");
			fprintf(stderr, "\nHelp: %s -t <topology-file-name> -i <routing-update-interval>\n\n", argv[0]);
			exit(EXIT_FAILURE);
		}
	}
	/* Checking if the user typed correct input command,if it is no then print "help" of the command. */
	if(interval <= 0 || fn == NULL || optind != argc) {
		fprintf(stderr, "\nIncomplete arguments, Please refer Help\n");
		fprintf(stderr, "\nHelp: %s -t <topology-file-name> -i <routing-update-interval>\n\n", argv[0]);
		exit(EXIT_FAILURE);
	}
	time_interval = interval;
	printf("\nDEBUG: Reading input file-name<%s> time_interval<%d>\n", fn, interval);

	/* read config from config file */
	if(load_config(fn) != 0) {
		fprintf(stderr, "The format of config-file %s is not correct\n", fn);
		exit(EXIT_FAILURE);
	}

	/* register sig alarm handler */
	signal(SIGALRM, alarm_handler);

	/* start the server */
	if(start_server() != 0) {
		exit(EXIT_FAILURE);
	}

	/* free resources */
	free_servers();
	return 0;
}
