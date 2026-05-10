import math
from collections import deque
from common_pb2 import Position

# ============================================================
# Unique tracked aruco
# ============================================================

class UniqueAruco:

    def __init__(
        self,
        x,
        y,
        theta,
        aruco_id,
        unique_id,
        frame,
        history_size=10
    ):

        # ---------- Filtered position ----------
        self.x = x
        self.y = y
        self.theta = theta

        # ---------- Raw measurements ----------
        self.raw_x = x
        self.raw_y = y
        self.raw_theta = theta

        # ---------- Position history ----------
        self.history_x = deque(maxlen=history_size)
        self.history_y = deque(maxlen=history_size)

        self.history_x.append(x)
        self.history_y.append(y)

        # ---------- IDs ----------
        self.aruco_id = aruco_id
        self.unique_id = unique_id

        # ---------- Tracking ----------
        self.last_seen = frame
        self.missed_frames = 0

        # ---------- Validation ----------
        self.hits = 1
        self.confirmed = False

    # ========================================================
    # Update with a new detection
    # ========================================================

    def update(self, x, y, theta, frame):

        # Save raw values
        self.raw_x = x
        self.raw_y = y
        self.raw_theta = theta

        # Add to history
        self.history_x.append(x)
        self.history_y.append(y)

        # ====================================================
        # Mean filter
        # ====================================================

        self.x = sum(self.history_x) / len(self.history_x)
        self.y = sum(self.history_y) / len(self.history_y)

        # ====================================================
        # Angle filter
        # ====================================================

        old_cos = math.cos(self.theta)
        old_sin = math.sin(self.theta)

        new_cos = math.cos(theta)
        new_sin = math.sin(theta)

        alpha = 0.1

        filt_cos = alpha * new_cos + (1 - alpha) * old_cos
        filt_sin = alpha * new_sin + (1 - alpha) * old_sin

        self.theta = math.atan2(filt_sin, filt_cos)

        # ====================================================

        self.last_seen = frame
        self.missed_frames = 0

        self.hits += 1

        if self.hits >= 3:
            self.confirmed = True

    # ========================================================
    # Distance to a detection
    # ========================================================

    def distance_to(self, x, y):

        dx = x - self.x
        dy = y - self.y

        return math.sqrt(dx * dx + dy * dy)

    # ========================================================

    def __repr__(self):

        return (
            f"[UID={self.unique_id}] "
            f"x={self.x:.1f} "
            f"y={self.y:.1f} "
            f"theta={math.degrees(self.theta):.1f}° "
            f"missed={self.missed_frames}"
        )


# ============================================================
# Main tracker
# ============================================================

class ArucoFilter:

    def __init__(self):

        self.tracks = []

        self.current_frame = 0

        self.next_unique_id = 0

        # ====================================================
        # Parameters
        # ====================================================

        self.MAX_ASSOCIATION_DISTANCE = 125.0  # mm

        self.TIMEOUT = 15

    # ========================================================
    # Main update
    # ========================================================

    def update(self, pos, ids_to_send):

        self.current_frame += 1

        matched_track_ids = set()

        unmatched_detections = []

        # ========================================================
        # Try associating detections
        # ========================================================

        for detection, aruco_id in zip(pos, ids_to_send):

            best_track = None
            best_distance = self.MAX_ASSOCIATION_DISTANCE

            for track in self.tracks:

                # Already matched this frame
                if track.unique_id in matched_track_ids:
                    continue

                d = track.distance_to(
                    detection.x,
                    detection.y
                )

                if d < best_distance:

                    best_distance = d
                    best_track = track

            # ====================================================
            # Match found
            # ====================================================

            if best_track is not None:

                best_track.update(
                    detection.x,
                    detection.y,
                    detection.theta,
                    self.current_frame
                )

                matched_track_ids.add(best_track.unique_id)

            # ====================================================
            # Save unmatched detection
            # ====================================================

            else:

                unmatched_detections.append(
                    (detection, aruco_id)
                )

        # ========================================================
        # Increment missed frames
        # ========================================================

        for track in self.tracks:

            if track.unique_id not in matched_track_ids:
                track.missed_frames += 1

        # ========================================================
        # Delete timed-out tracks
        # ========================================================

        self.tracks = [
            t for t in self.tracks
            if t.missed_frames < self.TIMEOUT
        ]

        # ========================================================
        # Create NEW tracks ONLY after cleanup
        # ========================================================

        for detection, aruco_id in unmatched_detections:

            # ================================================
            # Avoid duplicates
            # ================================================

            duplicate = False

            for track in self.tracks:

                d = track.distance_to(
                    detection.x,
                    detection.y
                )

                if d < self.MAX_ASSOCIATION_DISTANCE * 0.5:

                    duplicate = True
                    break

            if duplicate:
                continue

            # ================================================
            # Create track
            # ================================================

            new_track = UniqueAruco(
                detection.x,
                detection.y,
                detection.theta,
                aruco_id,
                self.next_unique_id,
                self.current_frame
            )

            self.next_unique_id += 1

            self.tracks.append(new_track)
    # ========================================================
    # Return confirmed tracks only
    # ========================================================

    def get_confirmed_tracks(self):

        return [
            t for t in self.tracks
            if t.confirmed
        ]

    # ========================================================
    # Debug print
    # ========================================================

    def print_tracks(self):

        print("\n================ TRACKS ================")

        for t in self.tracks:
            print(t)

        print("========================================\n")
        
    
    def get_filtered_detections(self):

        pos = []
        ids_to_send = []

        for track in self.get_confirmed_tracks():

            tag = Position(
                x=track.x,
                y=track.y,
                theta=track.theta
            )

            pos.append(tag)

            ids_to_send.append(track.aruco_id)

        return pos, ids_to_send


# ============================================================
# Example
# ============================================================

if __name__ == "__main__":

    tracker = ArucoFilter()

    # ========================================================
    # Fake detections
    # ========================================================

    for frame in range(30):

        pos = []
        ids_to_send = []

        # ====================================================
        # Object 1
        # ====================================================

        if frame < 20:

            x = 1000 + (math.sin(frame) * 15)
            y = 500 + (math.cos(frame) * 10)

            pos.append(Position(x, y, 0.0))
            ids_to_send.append(47)

        # ====================================================
        # Object 2
        # ====================================================

        if frame > 5:

            x = 1500 + (math.sin(frame * 0.5) * 20)
            y = 700 + (math.cos(frame * 0.5) * 15)

            pos.append(Position(x, y, 1.57))
            ids_to_send.append(47)

        # ====================================================

        tracker.update(pos, ids_to_send)

        print(f"FRAME {frame}")

        tracker.print_tracks()