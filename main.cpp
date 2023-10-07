#include <ranges>
#include <random>

#include <wx/wx.h>
#include <wx/sizer.h>
#include <wx/cmdline.h>
#include <wx/dialog.h>
#include <wx/wfstream.h>

#include "Time/Time.hpp"
#include "Image/Image.hpp"
#include "Image/File.hpp"
#include "StdIO/StdIO.hpp"
#include "Video/Video.hpp"

enum class DecoderType
{
    None = 0,
    STB,
    DirectXTex,
    FFMPEG,
    WxWidgets,
    OpenCv,
    Qt,
    GraphicsMagick,

    DECODER_MAX_ADD1
};

inline void AdjustConsoleBuffer(const int16_t minLength)
{
    CONSOLE_SCREEN_BUFFER_INFO conInfo;
    GetConsoleScreenBufferInfo(GetStdHandle(STD_OUTPUT_HANDLE), &conInfo);
    if (conInfo.dwSize.Y < minLength)
        conInfo.dwSize.Y = minLength;
    SetConsoleScreenBufferSize(GetStdHandle(STD_OUTPUT_HANDLE), conInfo.dwSize);
}

inline bool ReleaseConsole()
{
    bool result = true;
    FILE *fp;

    if (freopen_s(&fp, "NUL:", "r", stdin) != 0)
        result = false;
    else
        setvbuf(stdin, nullptr, _IONBF, 0);

    if (freopen_s(&fp, "NUL:", "w", stdout) != 0)
        result = false;
    else
        setvbuf(stdout, nullptr, _IONBF, 0);

    if (freopen_s(&fp, "NUL:", "w", stderr) != 0)
        result = false;
    else
        setvbuf(stderr, nullptr, _IONBF, 0);

    if (!FreeConsole())
        result = false;

    return result;
}

inline bool RedirectConsoleIO()
{
    bool result = true;
    FILE *fp;

    if (GetStdHandle(STD_INPUT_HANDLE) != INVALID_HANDLE_VALUE)
    {
        if (freopen_s(&fp, "CONIN$", "r", stdin) != 0)
            result = false;
        else
            setvbuf(stdin, NULL, _IONBF, 0);
    }

    if (GetStdHandle(STD_OUTPUT_HANDLE) != INVALID_HANDLE_VALUE)
    {
        if (freopen_s(&fp, "CONOUT$", "w", stdout) != 0)
            result = false;
        else
            setvbuf(stdout, NULL, _IONBF, 0);
    }

    if (GetStdHandle(STD_ERROR_HANDLE) != INVALID_HANDLE_VALUE)
    {
        if (freopen_s(&fp, "CONOUT$", "w", stderr) != 0)
            result = false;
        else
            setvbuf(stderr, NULL, _IONBF, 0);
    }

    std::ios::sync_with_stdio(true);

    std::wcout.clear();
    std::cout.clear();
    std::wcerr.clear();
    std::cerr.clear();
    std::wcin.clear();
    std::cin.clear();

    return result;
}

inline bool CreateNewConsole(const int16_t minLength)
{
    bool result = false;

    ReleaseConsole();

    if (AllocConsole())
    {
        AdjustConsoleBuffer(minLength);
        result = RedirectConsoleIO();
    }

    return result;
}

class wxImagePanel : public wxPanel
{
    std::optional<wxImage> image;

    ssize_t idx;
    std::vector<std::filesystem::path> images;

    wxBitmap resized;
    int w, h;

    wxFrame *parent;

    std::optional<CuVid::DecoderRGBA> vidCtx{};
    int frameCount = 0;
    std::function<void()> requestReadNextFrame = nullptr;

    std::array<DecoderType, static_cast<size_t>(DecoderType::DECODER_MAX_ADD1) - 1> CurrentDecoders = {};
    int currentDecodersIdx = 0;

public:
    explicit wxImagePanel(wxFrame *parent, std::function<void()> &&requestReadNextFrame) : wxPanel(parent), idx(-1), w(-1), h(-1), parent(parent), requestReadNextFrame(requestReadNextFrame)
    {
    }

private:
    std::optional<wxImage> TryLoadByWx(const std::filesystem::path &path) const
    {
        CuConsole::SetForegroundColor(CuConsole::Color::Gray);
        CuConsole::WriteLine("use wxWidgets");
        try
        {
            wxLogNull ignore{};
            return CuImg::LoadFile_DiscreteImageRGBA_wxWidgets(path).GetContext().Raw();
        }
        catch (const std::exception& ex)
        {
            CuConsole::SetForegroundColor(CuConsole::Color::Red);
            CuConsole::WriteLine("wxWidgets: ", ex.what());
        }
        return {};
    }

    std::optional<wxImage> TryLoadByStb(const std::filesystem::path &path) const
    {
        CuConsole::SetForegroundColor(CuConsole::Color::Gray);
        CuConsole::WriteLine("use stb");
        try
        {
            return CuImg::ConvertToDiscreteImageRGBA_wxWidgets(CuImg::LoadFile_ImageRGBA_STB(path)).GetContext().Raw();
        }
        catch (const std::exception &ex)
        {
            CuConsole::SetForegroundColor(CuConsole::Color::Red);
            CuConsole::WriteLine("stb: ", ex.what());
        }
        return {};
    }

    std::optional<wxImage> TryLoadByDirectXTex(const std::filesystem::path &path) const
    {
        CuConsole::SetForegroundColor(CuConsole::Color::Gray);
        CuConsole::WriteLine("use DirectXTex");
        try
        {
            const auto img = CuImg::LoadFile_ImageRGBA_DirectXTex(path);
            if (!img.GetInfo().WicProps.empty())
            {
                const auto leftSize = std::ranges::max(
                    img.GetInfo().WicProps | std::views::keys | std::views::transform([](const auto &x)
                                                                                      { return CuEnum::ToString<CuImg::DirectXTexContext::PhotoProp>(x).length(); }));
                for (const auto &[k, v] : img.GetInfo().WicProps)
                {
                    const auto right = CuStr::ToU8String(std::visit([&]<typename Tr>(const Tr &val)
                                                                    {
					                                                     using T = std::decay_t<Tr>;
					                                                     if constexpr (std::is_same_v<
						                                                     T, CuImg::DirectXTexContext::VtTime>)
					                                                     {
						                                                     const auto t =
							                                                     std::chrono::system_clock::to_time_t(
								                                                     std::chrono::clock_cast<
									                                                     std::chrono::system_clock>(
									                                                     val));
						                                                     tm local{};
                                                                             CuTime::Local(&local, &t);
						                                                     return CuStr::FormatU8("{}", 
							                                                     std::put_time(&local, "%F %X"));
					                                                     }
					                                                     else
					                                                     {
						                                                     return CuStr::FormatU8("{}", val);
					                                                     } },
                                                                    v));
                    const auto left = CuStr::PadRightU8(CuStr::ToU8String(CuEnum::ToString(k)), leftSize + 1, ' ');
                    CuConsole::SetForegroundColor(CuConsole::Color::Gray);
                    CuConsole::WriteLine(CuStr::ToDirtyUtf8StringView(CuStr::AppendsU8(left, right)));
                }
            }
            return CuImg::ConvertToDiscreteImageRGBA_wxWidgets(img).Raw();
        }
        catch (const std::exception &ex)
        {
            CuConsole::SetForegroundColor(CuConsole::Color::Red);
            CuConsole::WriteLine("DirectX Tex: ", ex.what());
        }
        return {};
    }

    std::optional<wxImage> TryLoadByFFmpeg(const std::filesystem::path &path, const bool dumpInfo = true)
    {
        CuConsole::SetForegroundColor(CuConsole::Color::Gray);
        CuConsole::WriteLine("use FFmpeg");
        try
        {
            CuVid::DecoderRGBA dec{};
            dec.Config.Input = path;
            wxImage res{};
            dec.Config.VideoHandler = [&](CuVid::DecoderRGBA::VideoFrameType &vf)
            {
                // res = RGBA2RGB_A(vf.Data(), vf.Width(), vf.Height(), vf.Linesize());
                res = CuImg::ConvertToDiscreteImageRGBA_wxWidgets(vf).GetContext().Raw();
            };
            dec.LoadFile();
            dec.FindStream();
            if (dumpInfo)
                av_dump_format(dec.GetFormatContext(), dec.Config.VideoIndex, reinterpret_cast<const char *>(path.u8string().c_str()), 0);
            while (!dec.Eof())
            {
                if (dec.Read() == CuVid::StreamTypeVideo)
                {
                    vidCtx = std::move(dec);
                    frameCount = 1;
                    return res;
                }
            }
        }
        catch (const std::exception &ex)
        {
            CuConsole::SetForegroundColor(CuConsole::Color::Red);
            CuConsole::WriteLine("FFmpeg: ", ex.what());
        }
        return {};
    }

    std::optional<wxImage> TryLoadByOpenCv(const std::filesystem::path &path)
    {
        CuConsole::SetForegroundColor(CuConsole::Color::Gray);
        CuConsole::WriteLine("use OpenCV");
        try
        {
            return CuImg::ConvertToDiscreteImageRGBA_wxWidgets(CuImg::LoadFile_ImageRGBA_OpenCV(path)).GetContext().Raw();
        }
        catch (const std::exception &ex)
        {
            CuConsole::SetForegroundColor(CuConsole::Color::Red);
            CuConsole::WriteLine("OpenCV: ", ex.what());
        }
        return {};
    }

    std::optional<wxImage> TryLoadByQt(const std::filesystem::path &path)
    {
        CuConsole::SetForegroundColor(CuConsole::Color::Gray);
        CuConsole::WriteLine("use Qt");
        try
        {
            return CuImg::ConvertToDiscreteImageRGBA_wxWidgets(CuImg::LoadFile_ImageRGBA_QT(path)).GetContext().Raw();
        }
        catch (const std::exception &ex)
        {
            CuConsole::SetForegroundColor(CuConsole::Color::Red);
            CuConsole::WriteLine("Qt: ", ex.what());
        }
        return {};
    }

    std::optional<wxImage> TryLoadByGraphicsMagick(const std::filesystem::path &path)
    {
        CuConsole::SetForegroundColor(CuConsole::Color::Gray);
        CuConsole::WriteLine("use GraphicsMagick");
        try
        {
            return CuImg::ConvertToDiscreteImageRGBA_wxWidgets(CuImg::LoadFile_DiscreteImageRGBOpacity16_GraphicsMagick(path)).GetContext().Img;
        }
        catch (const std::exception &ex)
        {
            CuConsole::SetForegroundColor(CuConsole::Color::Red);
            CuConsole::WriteLine("GraphicsMagick: ", ex.what());
        }
        return {};
    }

    std::optional<wxImage> TryLoadImpl(const std::filesystem::path &path)
    {
        switch (CurrentDecoders[currentDecodersIdx])
        {
        case DecoderType::STB:
        {
            if (auto r = TryLoadByStb(path); r)
                return r;
        }
        break;
        case DecoderType::DirectXTex:
        {
            if (auto r = TryLoadByDirectXTex(path); r)
                return r;
        }
        break;
        case DecoderType::FFMPEG:
        {
            if (auto r = TryLoadByFFmpeg(path); r)
                return r;
        }
        break;
        case DecoderType::WxWidgets:
        {
            if (auto r = TryLoadByWx(path); r)
                return r;
        }
        break;
        case DecoderType::OpenCv:
        {
            if (auto r = TryLoadByOpenCv(path); r)
                return r;
        }
        break;
        case DecoderType::Qt:
        {
            if (auto r = TryLoadByQt(path); r)
                return r;
            break;
        }
        case DecoderType::GraphicsMagick:
        {
            if (auto r = TryLoadByGraphicsMagick(path); r)
                return r;
            break;
        }
        default:
            throw std::runtime_error("unknown decoder");
        }
        return {};
    }

    std::optional<wxImage> TryLoad(const std::filesystem::path &path, const bool switchNextDecoder = false)
    {
        vidCtx.reset();

        CuConsole::SetForegroundColor(CuConsole::Color::White);
        CuConsole::WriteLine("=> ", CuStr::ToDirtyUtf8StringView(path.u8string()));
        switch (CuImg::GuessType(CuStr::ToLower(path.extension().string())))
        {
        case CuImg::ImageType::RAW:
            CurrentDecoders = {DecoderType::DirectXTex, DecoderType::FFMPEG, DecoderType::STB, DecoderType::Qt, DecoderType::OpenCv, DecoderType::WxWidgets, DecoderType::GraphicsMagick};
            break;
        default:
            CurrentDecoders = {DecoderType::FFMPEG, DecoderType::STB, DecoderType::Qt, DecoderType::OpenCv, DecoderType::WxWidgets, DecoderType::GraphicsMagick, DecoderType::DirectXTex};
            break;
        }

        currentDecodersIdx = switchNextDecoder ? currentDecodersIdx + 1 : 0;

        for (; currentDecodersIdx < CurrentDecoders.size(); ++currentDecodersIdx)
        {
            const auto load = [&]()
            {
                const auto start = std::chrono::high_resolution_clock::now();
                auto r = TryLoadImpl(path);
                const auto end = std::chrono::high_resolution_clock::now();
                const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
                CuConsole::SetForegroundColor(CuConsole::Color::Gray);
                CuConsole::WriteLine("try in ", ms.count(), "ms");
                return r;
            };
            if (auto r = load(); r)
            {
                SetForegroundColor(CuConsole::Color::White);
                CuConsole::WriteLine("done");
                return r;
            }
        }

        return {};
    }

    void SetImages(const std::filesystem::path &dir, const bool rec)
    {
        images.clear();
        if (rec)
        {
            for (const auto &p : std::filesystem::recursive_directory_iterator(dir))
            {
                if (p.is_regular_file())
                    images.push_back(p.path());
            }
        }
        else
        {
            for (const auto &p : std::filesystem::directory_iterator(dir))
            {
                if (p.is_regular_file())
                    images.push_back(p.path());
            }
        }

        std::ranges::sort(images);
        idx = -1;
    }

    void FindImage(const ssize_t offset = 0, const bool rev = false)
    {
        if (rev)
        {
            for (ssize_t i = offset; i >= 0; --i)
            {
                if (auto r = TryLoad(images[i]); r)
                {
                    idx = i;
                    image = std::move(r);
                    break;
                }
            }
        }
        else
        {
            for (size_t i = offset; i < images.size(); ++i)
            {
                if (auto r = TryLoad(images[i]); r)
                {
                    idx = i;
                    image = std::move(r);
                    break;
                }
            }
        }

        requestReadNextFrame();
    }

    static std::tuple<wxPoint, wxSize> GetImageRect(const int avaW, const int avaH, const int imgW, const int imgH)
    {
        float h = avaH;
        auto w = h * static_cast<float>(imgW) / static_cast<float>(imgH);
        if (w > avaW)
        {
            w = avaW;
            h = w * static_cast<float>(imgH) / static_cast<float>(imgW);
            return std::make_tuple(wxPoint(0, (avaH - h) / 2), wxSize(w, h));
        }
        return std::make_tuple(wxPoint((avaW - w) / 2, 0), wxSize(w, h));
    }

    void UpdateImage()
    {
        if (image && w > 0 && h > 0)
        {
            const auto [_, s] = GetImageRect(w, h, image->GetWidth(), image->GetHeight());
            resized = wxBitmap(image->Scale(s.x, s.y, wxIMAGE_QUALITY_NEAREST));
        }
        Refresh();
    }

public:
    void SetInput(const std::filesystem::path &path, const bool isFile)
    {
        if (isFile)
        {
            SetImages(path.parent_path(), false);
            for (size_t i = 0; i < images.size(); ++i)
            {
                if (images[i] == path)
                {
                    FindImage(i);
                    break;
                }
            }
        }
        else
        {
            SetImages(path, true);
            FindImage();
        }

        UpdateImage();
    }

    bool ReadNextFrame()
    {
        try
        {
            if (vidCtx)
            {
	            try
	            {
		            const auto handle = [&](auto &x)
		            {
		            	image = CuImg::ConvertToDiscreteImageRGBA_wxWidgets(x).Raw();
			            ++frameCount;
			            UpdateImage();
		            };
		            vidCtx->Config.VideoHandler = handle;
		            while (!vidCtx->Eof())
		            {
			            if (vidCtx->Read() == CuVid::StreamTypeVideo)
			            {
				            return true;
			            }
		            }
		            if (frameCount < 2)
			            return false;
		            image = TryLoadByFFmpeg(std::get<std::filesystem::path>(vidCtx->Config.Input), false);
		            UpdateImage();
		            return true;
	            }
	            catch (const std::exception& ex)
	            {
                    CuConsole::SetForegroundColor(CuConsole::Color::Red);
                    CuConsole::WriteLine(ex.what());
	            }
            }
        }
        catch (const std::exception &e)
        {
            MessageBox(nullptr, CuStr::ToWString(e.what()).c_str(), L"WTF", 0);
        }
        return false;
    }

    void paintEvent(wxPaintEvent &evt)
    {
        wxPaintDC dc(this);
        render(dc);
    }

    void paintNow()
    {
        wxClientDC dc(this);
        render(dc);
    }

    void OnSize(wxSizeEvent &event)
    {
        Refresh();
        event.Skip();
    }

    void render(wxDC &dc)
    {
        int neww, newh;
        dc.GetSize(&neww, &newh);

        if (!image)
            return;

        if (const auto [p, s] = GetImageRect(neww, newh, image->GetWidth(), image->GetHeight()); neww != w || newh != h)
        {
            resized = wxBitmap(image->Scale(s.x, s.y, wxIMAGE_QUALITY_NEAREST));
            w = neww;
            h = newh;
            dc.DrawBitmap(resized, p, false);
        }
        else
        {
            dc.DrawBitmap(resized, p, false);
        }

        static std::optional<std::string> oldTitle = {};
        if (const auto newTitle = CuStr::Combine(CuStr::ToDirtyUtf8StringView(images[idx].u8string()), " (", image->GetWidth(), "x", image->GetHeight(), ")"); newTitle != oldTitle)
        {
            parent->SetTitle(wxString::FromUTF8(newTitle));
            oldTitle = newTitle;
        }
    }

    void keyPressed(wxKeyEvent &event)
    {
        const auto key = event.GetKeyCode();

        switch (key)
        {
        case 'r':
        case 'R':
        {
            if (images.empty())
                return;

            const auto cp = images[idx];
            std::random_device rd{};
            std::ranges::shuffle(images, std::mt19937_64{rd()});
            for (size_t i = 0; i < images.size(); ++i)
            {
                if (images[i] == cp)
                {
                    idx = i;
                    break;
                }
            }
            wxMessageBox("random succeeded");
        }
        break;
        case 'o':
        case 'O':
        {
            if (wxFileDialog dlg(nullptr, _("Open image"), wxEmptyString, wxEmptyString,
                                 "Images (*.*)|*.*", wxFD_OPEN | wxDIALOG_NO_PARENT);
                dlg.ShowModal() != wxID_CANCEL)
            {
                SetInput(dlg.GetPath().ToStdWstring(), true);
            }
        }
        break;
        case WXK_RIGHT:
        case WXK_LEFT:
        {
            if (images.empty())
                return;

            if (key == WXK_RIGHT)
            {
                FindImage((idx + 1) % images.size(), false);
            }
            else
            {
                FindImage((images.size() + idx - 1) % images.size(), true);
            }
            UpdateImage();
            break;
        }

        case 'd':
        case 'D':
        {
            if (auto r = TryLoad(images[idx], true); r)
            {
                image = *r;
                requestReadNextFrame();
                UpdateImage();
            }
            else
            {
                wxMessageBox(L"not found");
            }
            break;
        }

        case 'c':
        case 'C':
        {
            AllocConsole();
            break;
        }

        default:
            break;
        }
    }

    bool hasConsole = false;

    void AllocConsole()
    {
        if (hasConsole)
            return;

        CreateNewConsole(1024);
        SetConsoleOutputCP(65001);
        ShowWindow(GetConsoleWindow(), true);

        av_log_set_level(AV_LOG_VERBOSE);
        hasConsole = true;
    }

    DECLARE_EVENT_TABLE()
};

BEGIN_EVENT_TABLE(wxImagePanel, wxPanel)
EVT_KEY_UP(wxImagePanel::keyPressed)
EVT_PAINT(wxImagePanel::paintEvent)
EVT_SIZE(wxImagePanel::OnSize)
END_EVENT_TABLE()

class MyApp : public wxApp
{
    wxFrame * frame = nullptr;
    wxImagePanel * drawPane = nullptr;

public:
    bool OnInit() override
    {
        CuImg::WxImageDiscreteContext::Init();

        auto* sizer = new wxBoxSizer(wxHORIZONTAL);
        frame = new wxFrame(nullptr, wxID_ANY, wxT("Image Viewer"), wxPoint(50, 50), wxSize(800, 600));
        frame->SetBackgroundColour(wxColour(14, 14, 14));
        frame->SetDoubleBuffered(true);

        drawPane = new wxImagePanel(frame, [&]
                                    { if (!renderLoopOn) SwitchRenderLoop(); });
        drawPane->DragAcceptFiles(true);

        drawPane->Connect(wxEVT_DROP_FILES, wxDropFilesEventHandler(MyApp::OnDropFiles), nullptr, this);
        drawPane->Layout();
        drawPane->Centre();

        sizer->Add(drawPane, 1, wxEXPAND);

        frame->SetSizer(sizer);

        frame->Show();

        if (argc > 1)
            WritePath(argv[1].ToStdWstring());
#ifdef _DEBUG
        else
        {
            CuImg::ImageRGBA_STB img(4, 3);
#define Sc(row, col, r, g, b, a) img.Set(row, col, CuImg::CuRGBA(r, g, b, a))
            Sc(0, 0, 255, 0, 0, 255);
            Sc(0, 1, 0, 255, 0, 255);
            Sc(0, 2, 0, 0, 255, 255);
            Sc(0, 3, 0, 0, 0, 255);
            Sc(1, 0, 127, 127, 127, 255);
            Sc(1, 1, 127, 127, 127, 255);
            Sc(1, 2, 127, 127, 127, 255);
            Sc(1, 3, 127, 127, 127, 127);
            Sc(2, 0, 0, 255, 255, 255);
            Sc(2, 1, 255, 0, 255, 255);
            Sc(2, 2, 255, 255, 0, 255);
            Sc(2, 3, 255, 255, 255, 0);

            constexpr auto tp = "./test.bmp";
            CuImg::SaveFile(tp, img);
            WritePath(tp);
        }
#endif

#ifdef _DEBUG
        drawPane->AllocConsole();
#endif

        return true;
    }

private:
    bool renderLoopOn = false;

public:
    void SwitchRenderLoop()
    {
        if (renderLoopOn)
            Disconnect(wxEVT_IDLE, wxIdleEventHandler(MyApp::OnIdle));
        else
            Connect(wxID_ANY, wxEVT_IDLE, wxIdleEventHandler(MyApp::OnIdle));
        renderLoopOn = !renderLoopOn;
    }

private:
    void OnIdle(wxIdleEvent &evt)
    {
        if (renderLoopOn)
        {
            if (drawPane->ReadNextFrame())
            {
                evt.RequestMore();
            }
            else
            {
                SwitchRenderLoop();
            }
        }
    }

    void WritePath(const std::filesystem::path &p) const
    {
        if (exists(p))
        {
            if (is_directory(p))
            {
                drawPane->SetInput(p, false);
            }
            else if (is_regular_file(p))
            {
                drawPane->SetInput(p, true);
            }
        }
    }

    void OnDropFiles(wxDropFilesEvent &event)
    {
        if (event.GetNumberOfFiles())
        {
            WritePath(event.GetFiles()[0].ToStdWstring());
        }
    }
};

IMPLEMENT_APP(MyApp);