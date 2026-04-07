# Pathfinding Visualizer

Dự án mô phỏng và trực quan hóa các thuật toán tìm đường (pathfinding) trên mê cung. Bao gồm giao diện web và chương trình C++ dòng lệnh.

## Tính năng

* Trực quan hóa quá trình tìm kiếm trên Web

* Hỗ trợ nhiều thuật toán:
  * Breadth-First Search (BFS)
  * Depth-First Search (DFS)
  * Uniform Cost Search (UCS)
  * Depth-Limited Search (DLS)
  * Iterative Deepening Search (IDS)
  * Greedy Best-First Search (GBFS)
  * A* Search
* So sánh hai thuật toán chạy song song
* Chạy từng bước (step-by-step)
* Hỗ trợ nhiều loại địa hình với chi phí khác nhau

## Yêu cầu môi trường:

* Web:
* Trình duyệt (Chrome, Edge, Firefox, etc)

## Cài đặt

*Tải 3 file:
```bash
index.html
script.js
style.css
```

## Cách sử dụng

* Chọn Web Visualizer
* để mở file html

```bash
open index.html
```

## Điều khiển:

*Thiết lập Lưới & Địa hình

- Grid Setup: Chỉnh số Hàng/Cột và số loại Địa hình (T1-T3). Nhấn Làm mới hoặc Ngẫu nhiên.
- Chi phí: Nhập giá trị điểm cho mỗi loại địa hình tại mục **Chi phí địa hình.

*Công cụ Vẽ

- Wall/Erase: Vẽ tường ngăn hoặc xóa.
- Start/End: Đặt điểm bắt đầu (xanh) và kết thúc (đỏ).
- T1, T2, T3: Vẽ các vùng di chuyển tốn phí (vàng, cam, nâu).

*Thuật toán & Thực thi

- Chọn: Chọn nhanh trên thanh ngang hoặc menu thả xuống (BFS, DFS, UCS, A*,...).
- Chạy: Tự động tìm đường.
  + Từng bước: Xem quy trình duyệt từng node.
  + Tốc độ: Gạt thanh trượt để chỉnh nhanh/chậm.
- Reset:Xóa path hoặc Xóa tất cả(trống lưới).

*Theo dõi kết quả

- Thống kê: Xem số node đã duyệt, tổng chi phí và chiều dài đường đi.
- So sánh: Chạy song song 2 thuật toán để đối chiếu hiệu suất.
---

## Chế độ:

* Mode 1: chạy 1 thuật toán
* Mode 2: chạy song song 2 thuật toán

## Cấu trúc dự án

```
project/
├── index.html        # Giao diện web
├── style.css         # CSS
├── script.js         # Logic + thuật toán
└── README.md
```

## Công nghệ sử dụng

* HTML / CSS / JavaScript
* Các thuật toán tìm kiếm AI

## Tác giả

- Trần Phúc Thái MSSV 202416603
- Kim Việt Tiến MSSV 202416618
- Nông Đức Toàn MSSV 202416622
- Phạm Mạnh Hùng MSSV 202416505

