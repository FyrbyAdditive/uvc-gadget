/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2025, Fyrby Additive Manufacturing & Engineering
 *
 * Servo Control Interface for UVC Pan/Tilt
 *
 * Communicates with servo_controller.py via HTTP
 *
 * THREADING CRITICAL: The uvc-gadget application runs a single-threaded
 * event loop that must respond to both video streaming (VS) and camera
 * control (CT) requests with minimal latency. Any blocking I/O in the 
 * main thread causes USB timeouts and stream failures.
 *
 * Solution: All HTTP POST requests for pan/tilt commands are dispatched
 * to detached background threads. This ensures the main event loop can
 * continue servicing video stream requests without interruption.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <errno.h>
#include <pthread.h>

#include "servo-control.h"

static char controller_url[256] = "http://localhost:8080";
static int initialized = 0;
static int persistent_sock = -1;
static struct sockaddr_in persistent_server;
static pthread_mutex_t sock_mutex = PTHREAD_MUTEX_INITIALIZER;
static int active_threads = 0;
static struct uvc_pantilt_absolute last_sent_position = {0, 0};
static int rate_limit_skip_count = 0;
#define MAX_ACTIVE_THREADS 2  /* Reduced from 5 - we want tight control */

/* Background thread for sending commands */
struct post_request {
	char path[128];
	char data[256];
};

static void *http_post_thread(void *arg)
{
	struct post_request *req = (struct post_request *)arg;
	char request[1024];
	int result;
	struct timeval timeout;

	pthread_mutex_lock(&sock_mutex);

	/* Recreate connection if needed */
	if (persistent_sock < 0) {
		persistent_sock = socket(AF_INET, SOCK_STREAM, 0);
		if (persistent_sock != -1) {
			/* Set socket timeouts to prevent hanging */
			timeout.tv_sec = 0;
			timeout.tv_usec = 100000;  /* 100ms timeout */
			setsockopt(persistent_sock, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
			setsockopt(persistent_sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

			/* Set socket options for low latency */
			int flag = 1;
			setsockopt(persistent_sock, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));

			/* Connect with timeout */
			if (connect(persistent_sock, (struct sockaddr *)&persistent_server, 
			            sizeof(persistent_server)) < 0) {
				perror("Servo control connect failed");
				close(persistent_sock);
				persistent_sock = -1;
				pthread_mutex_unlock(&sock_mutex);
				active_threads--;
				free(req);
				return NULL;
			}
		}
	}

	if (persistent_sock >= 0) {
		/* Send HTTP POST request with Connection: keep-alive */
		snprintf(request, sizeof(request),
			"POST %s HTTP/1.1\r\n"
			"Host: localhost:8080\r\n"
			"Content-Type: application/json\r\n"
			"Content-Length: %zu\r\n"
			"Connection: keep-alive\r\n"
			"\r\n"
			"%s", req->path, strlen(req->data), req->data);

		result = send(persistent_sock, request, strlen(request), MSG_NOSIGNAL);
		
		/* If send fails, reset connection for next time */
		if (result < 0) {
			perror("Servo control send failed");
			close(persistent_sock);
			persistent_sock = -1;
		}
	}

	pthread_mutex_unlock(&sock_mutex);
	active_threads--;
	free(req);
	return NULL;
}

/* Send HTTP GET request and read response */
static int http_get(const char *path, char *response, size_t response_size)
{
	int sock;
	struct sockaddr_in server;
	char request[512];
	int bytes_received;

	/* Create socket */
	sock = socket(AF_INET, SOCK_STREAM, 0);
	if (sock == -1) {
		perror("Could not create socket");
		return -1;
	}

	server.sin_addr.s_addr = inet_addr("127.0.0.1");
	server.sin_family = AF_INET;
	server.sin_port = htons(8080);

	/* Connect to server */
	if (connect(sock, (struct sockaddr *)&server, sizeof(server)) < 0) {
		perror("Connect failed");
		close(sock);
		return -1;
	}

	/* Send HTTP GET request */
	snprintf(request, sizeof(request),
		"GET %s HTTP/1.1\r\n"
		"Host: localhost:8080\r\n"
		"Connection: close\r\n"
		"\r\n", path);

	if (send(sock, request, strlen(request), 0) < 0) {
		perror("Send failed");
		close(sock);
		return -1;
	}

	/* Receive response */
	bytes_received = recv(sock, response, response_size - 1, 0);
	if (bytes_received < 0) {
		perror("Recv failed");
		close(sock);
		return -1;
	}

	response[bytes_received] = '\0';
	close(sock);

	return 0;
}

/* Send HTTP POST request - spawns thread to avoid blocking video stream */
static int http_post(const char *path, const char *data)
{
	pthread_t thread;
	struct post_request *req;

	/* Drop request if too many threads already active */
	if (active_threads >= MAX_ACTIVE_THREADS) {
		printf("WARNING: Dropping servo control request (too many active threads: %d)\n", active_threads);
		return -1;
	}

	/* Allocate request structure for thread */
	req = malloc(sizeof(struct post_request));
	if (!req)
		return -1;

	strncpy(req->path, path, sizeof(req->path) - 1);
	strncpy(req->data, data, sizeof(req->data) - 1);
	req->path[sizeof(req->path) - 1] = '\0';
	req->data[sizeof(req->data) - 1] = '\0';

	/* Spawn detached thread to send request - returns immediately */
	pthread_attr_t attr;
	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
	
	active_threads++;
	if (pthread_create(&thread, &attr, http_post_thread, req) != 0) {
		active_threads--;
		free(req);
		return -1;
	}
	
	pthread_attr_destroy(&attr);
	return 0;
}

int servo_control_init(const char *url)
{
	if (url)
		strncpy(controller_url, url, sizeof(controller_url) - 1);

	/* Setup server address for persistent connection */
	persistent_server.sin_addr.s_addr = inet_addr("127.0.0.1");
	persistent_server.sin_family = AF_INET;
	persistent_server.sin_port = htons(8080);

	initialized = 1;
	printf("Servo control initialized: %s\n", controller_url);
	return 0;
}

void servo_control_cleanup(void)
{
	pthread_mutex_lock(&sock_mutex);
	if (persistent_sock >= 0) {
		close(persistent_sock);
		persistent_sock = -1;
	}
	pthread_mutex_unlock(&sock_mutex);
	initialized = 0;
}

int servo_control_get_pantilt(struct uvc_pantilt_absolute *pantilt)
{
	if (!initialized)
		return -1;

	/* Return cached position instead of querying HTTP server
	 * This avoids blocking the UVC event loop on HTTP GET requests
	 * The position is updated by SET calls and reflects the target state */
	*pantilt = last_sent_position;
	return 0;
}

/* Query actual servo position from HTTP server - use sparingly */
int servo_control_query_pantilt(struct uvc_pantilt_absolute *pantilt)
{
	char response[2048];
	char *body;
	int pan_deg, tilt_deg;

	if (!initialized)
		return -1;

	/* Get current position from controller */
	if (http_get("/api/status", response, sizeof(response)) < 0)
		return -1;

	/* Find JSON body (after HTTP headers) */
	body = strstr(response, "\r\n\r\n");
	if (!body)
		return -1;
	body += 4;

	/* Parse JSON response: {"pan": 90, "tilt": 45} */
	if (sscanf(body, "{\"pan\": %d, \"tilt\": %d}", &pan_deg, &tilt_deg) != 2) {
		/* Try alternative format */
		char *pan_ptr = strstr(body, "\"pan\":");
		char *tilt_ptr = strstr(body, "\"tilt\":");
		
		if (!pan_ptr || !tilt_ptr)
			return -1;

		pan_deg = atoi(pan_ptr + 6);
		tilt_deg = atoi(tilt_ptr + 7);
	}

	/* Convert degrees to arc-seconds (degrees * 3600) */
	pantilt->pan = pan_deg * 3600;
	pantilt->tilt = tilt_deg * 3600;

	/* Update cached position */
	last_sent_position = *pantilt;

	return 0;
}

int servo_control_set_pantilt(const struct uvc_pantilt_absolute *pantilt)
{
	char data[256];
	int pan_deg, tilt_deg;

	if (!initialized)
		return -1;

	/* Rate limiting: Drop requests if threads are backed up */
	if (active_threads >= MAX_ACTIVE_THREADS) {
		rate_limit_skip_count++;
		if (rate_limit_skip_count % 10 == 1) {
			printf("WARNING: Dropping servo requests due to backlog (dropped %d so far)\n", 
			       rate_limit_skip_count);
		}
		/* Update local state anyway so GET_CUR returns the target */
		last_sent_position = *pantilt;
		return 0;  /* Pretend success to avoid error spam */
	}

	/* Skip duplicate requests - if position unchanged, don't send */
	if (pantilt->pan == last_sent_position.pan && 
	    pantilt->tilt == last_sent_position.tilt) {
		return 0;  /* Already at this position */
	}

	last_sent_position = *pantilt;
	rate_limit_skip_count = 0;  /* Reset skip counter on successful send */

	/* Convert arc-seconds to degrees */
	pan_deg = pantilt->pan / 3600;
	tilt_deg = pantilt->tilt / 3600;

	/* Prepare JSON data for /api/position endpoint */
	snprintf(data, sizeof(data),
		"{\"pan\": %d, \"tilt\": %d}",
		pan_deg, tilt_deg);

	/* Send absolute position command */
	return http_post("/api/position", data);
}

int servo_control_move_pantilt(const struct uvc_pantilt_relative *pantilt)
{
	char data[256];
	int pan_delta, tilt_delta;

	if (!initialized)
		return -1;

	/* Convert relative speed to delta (simplified) */
	/* UVC relative: -100 to +100, map to -10 to +10 degrees */
	pan_delta = (pantilt->pan * 10) / 100;
	tilt_delta = (pantilt->tilt * 10) / 100;

	/* Prepare JSON data for /api/move endpoint */
	snprintf(data, sizeof(data),
		"{\"pan\": %d, \"tilt\": %d}",
		pan_delta, tilt_delta);

	/* Send relative movement command */
	return http_post("/api/move", data);
}
