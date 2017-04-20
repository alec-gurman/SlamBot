#!/usr/bin/env python

class find_landmark(object):

    def __init__(self, red, green, blue):
        self.red_blobs = red
        self.green_blobs = green
        self.blue_blobs = blue
        self.landmark_cx = 0
        self.landmark_cy = 0
        self.landmark_area = 0
        self.landmark_marker = ((1,1),(1,1),1)


    def get_landmark_position(self, l_id):

        if l_id == 1:
            #pattern: (red, green, blue)
			for r_blob in self.red_blobs:
				highest_cy = r_blob[2]
				highest_cx = r_blob[1]
				for g_blob in self.green_blobs:
					current_cy = g_blob[2]
					current_cx = g_blob[1]
					if current_cy > highest_cy:
						if abs(current_cx - highest_cx) < 10:
							middle_cy = current_cy
							middle_cx = current_cx
							for b_blob in self.blue_blobs:
								current_cy = b_blob[2]
								current_cx = b_blob[1]
								if current_cy > middle_cy:
									if abs(current_cx - middle_cx) < 10:
										self.landmark_cx = current_cx
										self.landmark_cy = current_cy
										self.landmark_area = b_blob[0]
										self.landmark_marker = b_blob[7]

        elif l_id == 2:
            #pattern: (green, red, green)
			for g_blob in self.green_blobs:
				highest_cy = g_blob[2]
				highest_cx = g_blob[1]
				for r_blob in self.red_blobs:
					current_cy = r_blob[2]
					current_cx = r_blob[1]
					if current_cy > highest_cy:
						if abs(current_cx - highest_cx) < 10:
							middle_cy = current_cy
							middle_cx = current_cx
							for g_blob in self.green_blobs:
								current_cy = g_blob[2]
								current_cx = g_blob[1]
								if current_cy > middle_cy:
									if abs(current_cx - middle_cx) < 10:
										self.landmark_cx = middle_cx
										self.landmark_cy = middle_cy
										self.landmark_area = r_blob[0]
										self.landmark_marker = r_blob[7]
        elif l_id == 3:
            #pattern: (red, blue, red)
			for r_blob in self.red_blobs:
				highest_cy = r_blob[2]
				highest_cx = r_blob[1]
				for b_blob in self.blue_blobs:
					current_cy = b_blob[2]
					current_cx = b_blob[1]
					if current_cy > highest_cy:
						if abs(current_cx - highest_cx) < 10:
							middle_cy = current_cy
							middle_cx = current_cx
							for r_blob in self.red_blobs:
								current_cy = r_blob[2]
								current_cx = r_blob[1]
								if current_cy > middle_cy:
									if abs(current_cx - middle_cx) < 10:
										self.landmark_cx = middle_cx
										self.landmark_cy = middle_cy
										self.landmark_area = b_blob[0]
										self.landmark_marker = b_blob[7]

        return landmark_cx, landmark_cy, landmark_area, landmark_marker
