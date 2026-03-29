import os
import re
import cv2
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from matplotlib.widgets import Button, TextBox


RAW_DIR = os.path.join(os.path.dirname(__file__), "Raw_img")
OUT_DIR = os.path.join(os.path.dirname(__file__), "Cropped_img")


def natural_key(name: str):
    parts = re.split(r"(\d+)", name)
    return [int(p) if p.isdigit() else p.lower() for p in parts]


def order_points(pts: np.ndarray) -> np.ndarray:
    # 按极角排序，避免旋转矩形导致点顺序错误
    center = pts.mean(axis=0)
    angles = np.arctan2(pts[:, 1] - center[1], pts[:, 0] - center[0])
    order = np.argsort(angles)
    pts_sorted = pts[order]

    # 旋转到 top-left 作为起点
    sums = pts_sorted.sum(axis=1)
    start = int(np.argmin(sums))
    pts_sorted = np.roll(pts_sorted, -start, axis=0)

    # 确保顺时针顺序（tl, tr, br, bl）
    v1 = pts_sorted[1] - pts_sorted[0]
    v2 = pts_sorted[2] - pts_sorted[1]
    if np.cross(v1, v2) < 0:
        pts_sorted = np.array([pts_sorted[0], pts_sorted[3], pts_sorted[2], pts_sorted[1]])

    rect = np.zeros((4, 2), dtype="float32")
    rect[0] = pts_sorted[0]  # top-left
    rect[1] = pts_sorted[1]  # top-right
    rect[2] = pts_sorted[2]  # bottom-right
    rect[3] = pts_sorted[3]  # bottom-left
    return rect


def warp_quad(image: np.ndarray, pts: np.ndarray) -> np.ndarray:
    rect = order_points(pts.astype("float32"))
    (tl, tr, br, bl) = rect
    width_a = np.linalg.norm(br - bl)
    width_b = np.linalg.norm(tr - tl)
    height_a = np.linalg.norm(tr - br)
    height_b = np.linalg.norm(tl - bl)
    max_w = max(1, int(round(max(width_a, width_b))))
    max_h = max(1, int(round(max(height_a, height_b))))
    dst = np.array(
        [[0, 0], [max_w - 1, 0], [max_w - 1, max_h - 1], [0, max_h - 1]],
        dtype="float32",
    )
    m = cv2.getPerspectiveTransform(rect, dst)
    warped = cv2.warpPerspective(
        image,
        m,
        (max_w, max_h),
        flags=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_REPLICATE,
    )
    return warped


class QuadEditor:
    def __init__(self, images):
        self.images = images
        self.img_index = 0
        self.quad_index = 0
        self.points = []
        self.current_img = None
        self.drag_idx = None
        self.pick_radius = 12

        self.fig, self.ax = plt.subplots(figsize=(10, 7))
        self.fig.canvas.manager.set_window_title("裁切工具")
        plt.subplots_adjust(bottom=0.22)
        self.ax.axis("off")
        self.ax.set_aspect("equal", adjustable="box")

        self.image_artist = None
        self.point_artist, = self.ax.plot([], [], "o", color="yellow", markersize=8)
        self.line_artist, = self.ax.plot([], [], "-", color="lime", linewidth=2)
        self.poly_artist = Polygon(
            np.zeros((0, 2)),
            closed=True,
            facecolor=(0, 1, 0, 0.25),
            edgecolor="lime",
            linewidth=2,
            visible=False,
        )
        self.ax.add_patch(self.poly_artist)

        self._build_buttons()
        self._connect_events()
        self._load_image(0)

    def _build_buttons(self):
        btn_color = "#E8E8E8"
        ax_undo = self.fig.add_axes([0.04, 0.05, 0.10, 0.08])
        ax_reset = self.fig.add_axes([0.15, 0.05, 0.10, 0.08])
        ax_save = self.fig.add_axes([0.26, 0.05, 0.10, 0.08])
        ax_new = self.fig.add_axes([0.37, 0.05, 0.10, 0.08])
        ax_next = self.fig.add_axes([0.48, 0.05, 0.10, 0.08])
        ax_quit = self.fig.add_axes([0.59, 0.05, 0.10, 0.08])
        ax_goto = self.fig.add_axes([0.72, 0.05, 0.10, 0.08])
        ax_input = self.fig.add_axes([0.83, 0.05, 0.13, 0.08])

        self.btn_undo = Button(ax_undo, "Undo", color=btn_color, hovercolor="#D0D0D0")
        self.btn_reset = Button(ax_reset, "Reset", color=btn_color, hovercolor="#D0D0D0")
        self.btn_save = Button(ax_save, "Save", color=btn_color, hovercolor="#D0D0D0")
        self.btn_new = Button(ax_new, "New Quad", color=btn_color, hovercolor="#D0D0D0")
        self.btn_next = Button(ax_next, "Next", color=btn_color, hovercolor="#D0D0D0")
        self.btn_quit = Button(ax_quit, "Quit", color=btn_color, hovercolor="#D0D0D0")
        self.btn_goto = Button(ax_goto, "Go To", color=btn_color, hovercolor="#D0D0D0")
        self.text_goto = TextBox(ax_input, "", initial="")

        self.btn_undo.on_clicked(self._on_undo)
        self.btn_reset.on_clicked(self._on_reset)
        self.btn_save.on_clicked(self._on_save)
        self.btn_new.on_clicked(self._on_new_quad)
        self.btn_next.on_clicked(self._on_next)
        self.btn_quit.on_clicked(self._on_quit)
        self.btn_goto.on_clicked(self._on_goto)

    def _connect_events(self):
        self.fig.canvas.mpl_connect("button_press_event", self._on_click)
        self.fig.canvas.mpl_connect("button_press_event", self._on_drag_start)
        self.fig.canvas.mpl_connect("motion_notify_event", self._on_drag_move)
        self.fig.canvas.mpl_connect("button_release_event", self._on_drag_end)

    def _load_image(self, index):
        if index < 0 or index >= len(self.images):
            return
        img_path = os.path.join(RAW_DIR, self.images[index])
        img = cv2.imread(img_path)
        if img is None:
            return
        self.img_index = index
        self.current_img = img
        self.quad_index = 0
        self.points = []
        self.drag_idx = None

        rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        h, w = rgb.shape[:2]
        if self.image_artist is None:
            self.image_artist = self.ax.imshow(
                rgb,
                origin="upper",
                extent=(0, w, h, 0),
                interpolation="nearest",
            )
        else:
            self.image_artist.set_data(rgb)
            self.image_artist.set_extent((0, w, h, 0))
        self.ax.set_xlim(0, w)
        self.ax.set_ylim(h, 0)
        self.ax.set_aspect("equal", adjustable="box")
        self.ax.set_title(f"{self.images[index]}  |  使用鼠标点击4点，可拖动调整")
        self._update_artists()
        self.fig.canvas.draw_idle()

    def _update_artists(self):
        if self.points:
            xs, ys = zip(*self.points)
            self.point_artist.set_data(xs, ys)
        else:
            self.point_artist.set_data([], [])

        if len(self.points) >= 2:
            line_pts = self.points.copy()
            if len(self.points) == 4:
                line_pts.append(self.points[0])
            xs, ys = zip(*line_pts)
            self.line_artist.set_data(xs, ys)
        else:
            self.line_artist.set_data([], [])

        if len(self.points) == 4:
            self.poly_artist.set_xy(np.array(self.points))
            self.poly_artist.set_visible(True)
        else:
            self.poly_artist.set_visible(False)

    def _on_click(self, event):
        if event.inaxes != self.ax or event.button != 1:
            return
        if self.drag_idx is not None:
            return
        if len(self.points) < 4 and event.xdata is not None and event.ydata is not None:
            self.points.append((int(event.xdata), int(event.ydata)))
            self._update_artists()
            self.fig.canvas.draw_idle()

    def _nearest_point_index(self, x, y):
        if not self.points:
            return None
        dists = []
        for px, py in self.points:
            d = np.hypot(px - x, py - y)
            dists.append(d)
        min_idx = int(np.argmin(dists))
        return min_idx if dists[min_idx] <= self.pick_radius else None

    def _on_drag_start(self, event):
        if event.inaxes != self.ax or event.button != 1:
            return
        if event.xdata is None or event.ydata is None:
            return
        idx = self._nearest_point_index(event.xdata, event.ydata)
        if idx is not None:
            self.drag_idx = idx

    def _on_drag_move(self, event):
        if self.drag_idx is None:
            return
        if self.current_img is None:
            return
        # 允许在坐标轴边缘外拖动，使用像素坐标反推数据坐标
        if event.x is None or event.y is None:
            return
        xdata, ydata = self.ax.transData.inverted().transform((event.x, event.y))
        h, w = self.current_img.shape[:2]
        x = int(np.clip(xdata, 0, w - 1))
        y = int(np.clip(ydata, 0, h - 1))
        self.points[self.drag_idx] = (x, y)
        self._update_artists()
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

    def _on_drag_end(self, event):
        self.drag_idx = None

    def _on_undo(self, event):
        if self.points:
            self.points.pop()
            self._update_artists()
            self.fig.canvas.draw_idle()

    def _on_reset(self, event):
        self.points = []
        self._update_artists()
        self.fig.canvas.draw_idle()

    def _on_new_quad(self, event):
        self.points = []
        self._update_artists()
        self.fig.canvas.draw_idle()

    def _on_save(self, event):
        if len(self.points) != 4:
            return
        pts = np.array(self.points, dtype="float32")
        crop = warp_quad(self.current_img, pts)
        base_name = os.path.splitext(self.images[self.img_index])[0]
        out_name = f"{base_name}_quad_{self.quad_index:02d}.png"
        out_path = os.path.join(OUT_DIR, out_name)
        cv2.imwrite(out_path, crop)
        self.quad_index += 1

    def _on_next(self, event):
        next_index = self.img_index + 1
        if next_index >= len(self.images):
            plt.close(self.fig)
            return
        self._load_image(next_index)

    def _on_quit(self, event):
        plt.close(self.fig)

    def _on_goto(self, event):
        text = self.text_goto.text.strip()
        if not text:
            return
        if text in self.images:
            self._load_image(self.images.index(text))
            return
        if text.lower().endswith((".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff")):
            for i, name in enumerate(self.images):
                if name.lower() == text.lower():
                    self._load_image(i)
                    return
        if text.isdigit():
            idx = int(text) - 1
            if 0 <= idx < len(self.images):
                self._load_image(idx)


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    images = [
        f
        for f in os.listdir(RAW_DIR)
        if f.lower().endswith((".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff"))
    ]
    images.sort(key=natural_key)

    if not images:
        print("未找到图片文件。")
        return

    QuadEditor(images)
    plt.show()


if __name__ == "__main__":
    main()

