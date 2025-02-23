#include "SlamVisualizer.h"

#include <atomic>
#include <opencv2/opencv.hpp>
using namespace pangolin;

struct CustomType {
    CustomType() : x(0), y(0.0f) {}

    CustomType(int x, float y, std::string z) : x(x), y(y), z(z) {}

    int x;
    float y;
    std::string z;
};
std::atomic<bool> saveMvp(false);
std::atomic<bool> bRecord(false);
void SaveMvpMatrixMethod() { saveMvp.store(true); }
void RecordVideo() { bRecord.store(true); }

std::ostream& operator<<(std::ostream& os, const CustomType& o) {
    os << o.x << " " << o.y << " " << o.z;
    return os;
}

std::istream& operator>>(std::istream& is, CustomType& o) {
    is >> o.x;
    is >> o.y;
    is >> o.z;
    return is;
}

pangolin::Var<bool> FollowCamera("ui.A_Button", false, true);
std::vector<pangolin::Viewport> v2Save;

void SlamVisualizer::FinishFrame() {
    if (flag_frame_by_frame_) {
        std::unique_lock<std::mutex> lck(mtx_frame_by_frame_);
        cv_frame_by_frame_.wait(lck);
    }
}

void SlamVisualizer::PushViewMatrix(std::vector<FrameGL>& v_Frames) {
    std::lock_guard<std::mutex> lck(mtx_frame_);
    std::for_each(frames_.begin(), frames_.end(),
                  [](FrameGL& frame) { frame.type = 0; });
    for (auto& frame : v_Frames) {
        if (frame.m_id >= int(frames_.size())) {
            frames_.reserve(frame.m_id * 1.5);
            frames_.resize(frame.m_id + 1);
        }
        frames_[frame.m_id] = std::move(frame);
    }
    T_wc = OpenGlMatrix::ColMajor4x4(frames_.back().Twc.data());
}

void SlamVisualizer::PushImageTexture(unsigned char* imageTexture,
                                      const int width, const int height,
                                      const int channels,
                                      const std::string& name) {
    std::lock_guard<std::mutex> lck(mtx_image_texture_);
    if (height_ * width_ * channels_ < width * height * channels) {
        height_ = height;
        width_ = width;
        channels_ = channels;
        delete[] image_texture_;
        image_texture_ = new unsigned char[width_ * height_ * channels_];
    }

    memcpy(image_texture_, imageTexture, width_ * height_ * channels_);
}

void SlamVisualizer::PushWorldPoint(
    const std::vector<WorldPointGL>& v_Point3f) {
    std::lock_guard<std::mutex> lck(mtx_point_);

    for (auto& p : v_Point3f) {
        if (p.m_id * 3 > num_point_buffer_size_) {
            num_point_buffer_size_ = int(p.m_id * 4);
            world_points_.resize(num_point_buffer_size_);
        }

        world_points_[p.m_id * 3] = p.P.x();
        world_points_[p.m_id * 3 + 1] = p.P.y();
        world_points_[p.m_id * 3 + 2] = p.P.z();
        if (p.m_id >= point_size_) point_size_ = p.m_id + 1;
    }
}

SlamVisualizer::SlamVisualizer(int width, int height)
    : width_(width), height_(height) {
    channels_ = 3;
    image_texture_ = new unsigned char[height_ * width_ * channels_];
    num_point_buffer_size_ = 1000;
    world_points_.resize(num_point_buffer_size_);
    point_size_ = 0;
    flag_frame_by_frame_.store(false);
}

SlamVisualizer::~SlamVisualizer() { delete[] image_texture_; }
// float view[16] = { -0.770377,       0.124414,        -0.625332, 213.769,
//-0.590902,       0.229077,        0.773537,        1396.4,
// 0.239488,        0.965425 ,       -0.102959  ,     -5354.47,
// 0    ,   0      , 0     ,  1 };

float view[16] = {0.970367,    0.033256,    0.239337,      0.000000,
                  0.075530,    -0.982599,   -0.169694,     0.000000,
                  0.229529,    0.182742,    -0.955993,     0.000000,
                  6075.697813, -939.783107, -11454.347908, 1.000000};
void SlamVisualizer::render() {
    const int UI_WIDTH = 180;
    // Create OpenGL window in single line
    pangolin::CreateWindowAndBind("SlamVisualizer", width_ * 2 + UI_WIDTH,
                                  height_);
    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    OpenGlMatrix v = OpenGlMatrix::ColMajor4x4(view);
    // v = v.Transpose();
    //  Define Camera Render Object (for view / scene browsing)
    view_matrix_.SetProjectionMatrix(
        pangolin::ProjectionMatrix(640, 480, 420, 420, 320, 240, 0.1, 100000));
    // view_matrix_.SetModelViewMatrix(pangolin::ModelViewLookAt(-0, 0, -100, 0,
    // 0, 0, pangolin::AxisY));
    view_matrix_.SetModelViewMatrix(v);

    follow_matrix_ = view_matrix_;

    pangolin::Handler3D handler3D(view_matrix_);

    View& view3D = pangolin::Display("Geometry");

    view3D
        .SetBounds(0., 1., pangolin::Attach::Pix(UI_WIDTH),
                   pangolin::Attach::Pix(UI_WIDTH + width_), 640. / 480.)
        .SetHandler(&handler3D);

    View& viewImageTexture = pangolin::Display("ImageTexture");

    viewImageTexture
        .SetBounds(0., 1., pangolin::Attach::Pix(UI_WIDTH + width_), 1.,
                   -640. / 480.)
        .SetLock(pangolin::LockRight, pangolin::LockBottom);

    // Add named Panel and bind to variables beginning 'ui'
    // A Panel is just a View with a default layout and input handling
    pangolin::CreatePanel("ui").SetBounds(0.0, 1.0, 0.0,
                                          pangolin::Attach::Pix(UI_WIDTH));

    // Safe and efficient binding of named variables.
    // Specialisations mean no conversions take place for exact types
    // and conversions between scalar types are cheap.

    pangolin::Var<double> a_double("ui.A_Double", 3, 0, 5);
    pangolin::Var<int> an_int("ui.An_Int", 2, 0, 5);
    pangolin::Var<double> a_double_log("ui.Log_scale var", 3, 1, 1E4, true);
    pangolin::Var<bool> a_checkbox("ui.A_Checkbox", false, true);
    pangolin::Var<int> an_int_no_input("ui.An_Int_No_Input", 2);
    pangolin::Var<CustomType> any_type("ui.Some_Type",
                                       CustomType(0, 1.2f, "Hello"));

    pangolin::Var<bool> save_window("ui.Save_Window", false, false);
    pangolin::Var<bool> save_cube("ui.Save_Cube", false, false);

    pangolin::Var<bool> record_cube("ui.Record_Cube", false, false);

    // std::function objects can be used for Var's too. These work great with
    // C++11 closures.
    pangolin::Var<std::function<void(void)> > Reset("ui.RecordVideo",
                                                    RecordVideo);

    // Demonstration of how we can register a keyboard hook to alter a Var
    pangolin::RegisterKeyPressCallback(
        'f', pangolin::ToggleVarFunctor("ui.A_Button"));

    // Demonstration of how we can register a keyboard hook to trigger a method
    pangolin::RegisterKeyPressCallback('s', SaveMvpMatrixMethod);

    pangolin::RegisterKeyPressCallback('d', [this]() {
        std::unique_lock<std::mutex> lck(mtx_frame_by_frame_);
        cv_frame_by_frame_.notify_all();
    });

    pangolin::RegisterKeyPressCallback('e', [this]() {
        if (flag_frame_by_frame_)
            flag_frame_by_frame_.store(false);
        else
            flag_frame_by_frame_.store(true);
    });

    while (!pangolin::ShouldQuit()) {
        _clear();

        _drawImageTexture();

        _drawGeometry();

        // Swap frames and Process Events
        pangolin::FinishFrame();

        _saveFrames();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

void SlamVisualizer::_drawImageTexture() {
    std::lock_guard<std::mutex> lck(mtx_image_texture_);
    glColor3f(1.0, 1.0, 1.0);
    // Activate efficiently by object
    gl_texture_.Reinitialise(width_, height_, GL_RGB, 0, 0, GL_BGR,
                             GL_UNSIGNED_BYTE, image_texture_);

    pangolin::Display("ImageTexture").Activate();
    gl_texture_.RenderToViewportFlipY();
    if (bRecord) {
        auto& textView = pangolin::Display("ImageTexture");
        const Viewport tosave = textView.v.Intersect(textView.vp);
        v2Save.push_back(tosave);
    }
}

void SlamVisualizer::_drawGeometry() {
    glColor3f(1.0, 1.0, 1.0);

    pangolin::Display("Geometry").Activate();
    if (FollowCamera.Get()) {
        follow_matrix_.SetModelViewMatrix(view_matrix_.GetModelViewMatrix() *
                                          T_wc.Inverse());

    } else {
        follow_matrix_.SetModelViewMatrix(view_matrix_.GetModelViewMatrix());
    }
    if (saveMvp) {
        std::cout << follow_matrix_.GetModelViewMatrix() << std::endl;
        saveMvp.store(false);
    }
    follow_matrix_.Apply();
    _drawWorldAxis();

    glDrawColouredCube(-50, 50);

    _drawWorldPoints();
    _drawCameraFrustum();
    // mvp.Apply();
    if (bRecord) {
        auto& textView = pangolin::Display("Geometry");
        const Viewport tosave = textView.v.Intersect(textView.vp);
        v2Save.push_back(tosave);
        bRecord.store(0);
    }
}

void SlamVisualizer::_clear() {
    // Clear entire screen
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void SlamVisualizer::_drawWorldPoints() {
    std::lock_guard<std::mutex> lck(mtx_point_);

    glPointSize(5);
    glEnableClientState(GL_VERTEX_ARRAY);
    glColor4f(1, 0, 0, 1);
    glVertexPointer(3, GL_FLOAT, 0, &world_points_[0]);
    glDrawArrays(GL_POINTS, 0, point_size_);
    glDisableClientState(GL_VERTEX_ARRAY);
}

void SlamVisualizer::_drawCameraFrustum() {
    std::lock_guard<std::mutex> lck(mtx_frame_);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    for (int i = 1, n = frames_.size(); i < n; ++i) {
        glLoadIdentity();
        glMultMatrixd((follow_matrix_.GetModelViewMatrix() *
                       OpenGlMatrix::ColMajor4x4(frames_[i].Twc.data()))
                          .m);
        if (frames_[i].type == 1)
            glColor4f(0, 0, 1, 1);
        else
            glColor4f(0, 1, 0, 1);
        pangolin::glDrawFrustum(-2, -1.5, 1., 1., 4, 3, 8.f);
    }
    glPopMatrix();
}

void SlamVisualizer::Start() {
    thread_ = new std::thread(std::mem_fn(&SlamVisualizer::render), this);
}

void SlamVisualizer::Join() {
    if (thread_->joinable()) {
        thread_->join();
    }
}

void SlamVisualizer::Stop() {}

void SlamVisualizer::detach() { thread_->detach(); }
const float world_axis[] = {0, 0,   0, 100, 0, 0, 0, 0, 0,
                            0, 100, 0, 0,   0, 0, 0, 0, 100};

void SlamVisualizer::_drawWorldAxis() {
    glDrawAxis(100);
    // glLineWidth(3);
    //   glEnableClientState(GL_VERTEX_ARRAY);
    //   glColor4f(1.0f, 0.0f, 0.0f, 1.0f);
    //   glVertexPointer(3, GL_FLOAT, 0, world_axis);
    //   glDrawArrays(GL_LINES, 0, 2);
    //   glColor4f(0.0f, 1.0f, 0.0f, 1.0f);
    //   glVertexPointer(3, GL_FLOAT, 0, world_axis + 6);
    //   glDrawArrays(GL_LINES, 0, 2);
    //   glColor4f(0.0f, 0.0f, 1.0f, 1.0f);
    //   glVertexPointer(3, GL_FLOAT, 0, world_axis + 12);
    //   glDrawArrays(GL_LINES, 0, 2);
    //   glDisableClientState(GL_VERTEX_ARRAY);
}

void SlamVisualizer::_saveFrames() {
    if (v2Save.size() != 2) {
        return;
    }
    pangolin::Viewport& v1 = v2Save.front();
    pangolin::Viewport& v2 = v2Save.back();

    static cv::VideoWriter video1("VideoTest.avi",
                                  CV_FOURCC('M', 'J', 'P', 'G'), 25.0,
                                  cv::Size(v1.w, v1.h));
    static cv::VideoWriter video2("VideoTest2.avi",
                                  CV_FOURCC('M', 'J', 'P', 'G'), 25.0,
                                  cv::Size(v2.w, v2.h));

    cv::Mat mat1(v1.h, v1.w, CV_8UC3);
    glReadBuffer(GL_BACK);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);  // TODO: Avoid this?
    glReadPixels(v1.l, v1.b, v1.w, v1.h, GL_BGR, GL_UNSIGNED_BYTE, mat1.data);
    video1 << mat1;
    cv::Mat mat2(v2.h, v2.w, CV_8UC3);
    glReadBuffer(GL_BACK);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);  // TODO: Avoid this?
    glReadPixels(v2.l, v2.b, v2.w, v2.h, GL_BGR, GL_UNSIGNED_BYTE, mat2.data);
    video2 << mat2;
}
