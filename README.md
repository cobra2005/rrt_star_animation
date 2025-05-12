# Báo Cáo Tìm Đường Bằng RRT\*

## 1. Tổng Quan

Dự án này trình bày việc hiện thực thuật toán RRT\* (Rapidly-exploring Random Tree Star) để giải quyết bài toán tìm đường trong không gian 2D. Thuật toán tìm đường tối ưu từ điểm xuất phát đến điểm đích trong một bản đồ 2D có chứa các chướng ngại vật hình tròn.

## 2. Tóm Tắt Thuật Toán

RRT\* xây dựng một cây các đường đi hợp lệ bằng cách lấy mẫu ngẫu nhiên trong không gian. Ở mỗi vòng lặp, thuật toán:

-   Lấy mẫu một điểm trong không gian.
-   Tìm nút gần nhất trong cây hiện tại.
-   Di chuyển về phía điểm mẫu với một bước nhỏ (steering).
-   Kiểm tra va chạm với chướng ngại vật.
-   Tìm các nút lân cận để tối ưu kết nối (chọn nút cha có chi phí thấp nhất).
-   Rewire (kết nối lại) cây để cải thiện chất lượng đường đi.
-   Cập nhật đường đi tốt nhất tới đích nếu tìm được đường tốt hơn.

## 3. Các Đặc Điểm Chính Của Chương Trình

-   **Hiển thị trực tiếp**: Quá trình tìm kiếm và tối ưu đường đi được hiển thị trực tiếp theo thời gian thực.
-   **Cải thiện dần**: Sau khi tìm được đường đầu tiên, thuật toán vẫn tiếp tục để tìm đường tốt hơn.
-   **Rewire động**: Cập nhật lại các đường đi nếu phát hiện kết nối tốt hơn.
-   **Xử lý chướng ngại vật**: Chướng ngại vật là các hình tròn, kiểm tra va chạm được thực hiện bằng cách tính khoảng cách từ đoạn đường đến tâm chướng ngại vật.

## 4. Các Tham Số Sử Dụng

-   `max_iter=500`: Số vòng lặp tối đa.
-   `step_size=0.5`: Khoảng cách di chuyển mỗi lần mở rộng.
-   `search_radius=1.0`: Bán kính tìm kiếm nút lân cận để rewiring.
-   `goal_bias=0.1`: 10% mẫu sẽ bias về phía đích.

## 5. Hiển Thị Trực Quan

Mô phỏng bao gồm:

-   Điểm bắt đầu (màu xanh) và điểm đích (màu đỏ).
-   Chướng ngại vật là các vòng tròn màu xám.
-   Các cạnh cây là đường màu xanh lá.
-   Đường đi tốt nhất hiện tại là đường màu đỏ.

## 6. Cách Chạy Chương Trình

```
python rrt_star.py
```

Yêu cầu cài đặt `matplotlib` và `numpy`.

## 7. Các Cải Tiến

-   Hiển thị động theo từng vòng lặp.
-   Tự động cập nhật đường đi khi tìm được đường tốt hơn.
-   Tiếp tục tìm kiếm sau khi tìm được đường đi đầu tiên.

## 8. Kết Luận

Chương trình đã thể hiện hiệu quả cách thuật toán RRT\* tìm và cải thiện đường đi qua quá trình lấy mẫu, kết nối và rewiring lặp lại. Rất hữu ích cho các ứng dụng trong robot, điều hướng tự động, và các bài toán lập kế hoạch đường đi tối ưu.
