#include "rae_hw/peripherals/mic.hpp"

#include <vector>

#include "alsa/asoundlib.h"
#include "alsa/pcm.h"
#include "rclcpp/rclcpp.hpp"

namespace rae_hw {
MicNode::MicNode(const rclcpp::NodeOptions& options) : rclcpp_lifecycle::LifecycleNode("mic_node", options), handle_(nullptr) {}
MicNode::~MicNode() {
    cleanup();
}

void MicNode::cleanup() {
    snd_pcm_close(handle_);
}

CallbackReturn MicNode::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
    publisher_ = this->create_publisher<rae_msgs::msg::RAEAudio>("audio_in", 10);
    configure_microphone();

    timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&MicNode::timer_callback, this));
    wav_filename_ = "/tmp/recording.wav";
    start_service_ = this->create_service<rae_msgs::srv::RecordAudio>("start_recording",
                                                                      std::bind(&MicNode::startRecording, this, std::placeholders::_1, std::placeholders::_2));
    stop_service_ = this->create_service<rae_msgs::srv::StopRecording>("stop_recording",
                                                                       std::bind(&MicNode::stopRecording, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(this->get_logger(), "Mic node configured!");
    return CallbackReturn::SUCCESS;
}

CallbackReturn MicNode::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
    RCLCPP_INFO(this->get_logger(), "Mic node activated!");
    return CallbackReturn::SUCCESS;
}

CallbackReturn MicNode::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
    RCLCPP_INFO(this->get_logger(), "Mic node deactivated!");
    return CallbackReturn::SUCCESS;
}

CallbackReturn MicNode::on_shutdown(const rclcpp_lifecycle::State& /*previous_state*/) {
    RCLCPP_INFO(this->get_logger(), "Mic node shuttind down!");
    cleanup();
    return CallbackReturn::SUCCESS;
}

void MicNode::configure_microphone() {
    snd_pcm_hw_params_t* params;
    unsigned int sample_rate = 44100;
    int dir;

    int rc = snd_pcm_open(&handle_, "hw:0,1", SND_PCM_STREAM_CAPTURE, 0);
    if(rc < 0) {
        RCLCPP_FATAL(this->get_logger(), "Unable to open PCM device: %s", snd_strerror(rc));
        return;
    }
    recording_ = false;

    snd_pcm_hw_params_alloca(&params);
    snd_pcm_hw_params_any(handle_, params);
    snd_pcm_hw_params_set_access(handle_, params, SND_PCM_ACCESS_RW_INTERLEAVED);
    snd_pcm_hw_params_set_format(handle_, params, SND_PCM_FORMAT_S24_LE);
    snd_pcm_hw_params_set_channels(handle_, params, 2);
    snd_pcm_hw_params_set_rate_near(handle_, params, &sample_rate, &dir);

    rc = snd_pcm_hw_params(handle_, params);
    if(rc < 0) {
        RCLCPP_FATAL(this->get_logger(), "Unable to set HW parameters: %s", snd_strerror(rc));
        return;
    }
}

void MicNode::timer_callback() {
    if(recording_) {
        snd_pcm_uframes_t frames = 44100 * 0.05;
        std::vector<int32_t> buffer(2 * frames);

        int rc = snd_pcm_readi(handle_, buffer.data(), frames);
        if(rc == -EPIPE) {
            snd_pcm_prepare(handle_);
            RCLCPP_ERROR(this->get_logger(), "Overrun occurred. Preparing the interface.");
            return;
        } else if(rc < 0) {
            RCLCPP_ERROR(this->get_logger(), "Error from ALSA readi: %s", snd_strerror(rc));
            return;
        }

        rae_msgs::msg::RAEAudio msg;
        msg.header.stamp = this->now();
        msg.seq_num = seq_num_++;
        msg.frames = frames;
        msg.channels = 2;
        msg.sample_rate = 44100;
        msg.encoding = "S24LE";
        msg.is_bigendian = 0;
        msg.layout = msg.LAYOUT_INTERLEAVED;
        msg.step = 6;

        std::vector<int32_t> dataVec(buffer.data(), buffer.data() + 2 * frames * sizeof(int32_t));

        msg.data = dataVec;

        // Save the audio buffer to a WAV file
        saveToWav(buffer, frames);

        publisher_->publish(msg);
    }
}

void MicNode::applyLowPassFilter(std::vector<int32_t>& buffer) {
    static float prevSampleLeft = 0.0f;
    static float prevSampleRight = 0.0f;
    const float ALPHA = 0.1f;  // Smoothing factor

    for(size_t i = 0; i < buffer.size(); i += 2) {
        // Apply low-pass filter to attenuate high frequencies for left channel
        float currentSampleLeft = static_cast<float>(buffer[i]);
        float filteredSampleLeft = prevSampleLeft + ALPHA * (currentSampleLeft - prevSampleLeft);
        prevSampleLeft = filteredSampleLeft;
        buffer[i] = static_cast<int32_t>(filteredSampleLeft);

        // Apply low-pass filter to attenuate high frequencies for right channel
        float currentSampleRight = static_cast<float>(buffer[i + 1]);
        float filteredSampleRight = prevSampleRight + ALPHA * (currentSampleRight - prevSampleRight);
        prevSampleRight = filteredSampleRight;
        buffer[i + 1] = static_cast<int32_t>(filteredSampleRight);
    }
}

void MicNode::saveToWav(const std::vector<int32_t>& buffer, snd_pcm_uframes_t frames) {
    SF_INFO sfinfo;
    sfinfo.channels = 2;
    sfinfo.samplerate = 44100;
    sfinfo.format = SF_FORMAT_WAV | SF_FORMAT_PCM_24;

    // Open the file in read-write mode
    SNDFILE* file = sf_open(wav_filename_.c_str(), SFM_RDWR, &sfinfo);

    if(!file) {
        RCLCPP_ERROR(this->get_logger(), "Error opening WAV file for writing: %s", sf_strerror(file));
        return;
    }

    // Seek to the end of the file
    sf_count_t count = sf_seek(file, 0, SEEK_END);
    if(count < 0) {
        RCLCPP_ERROR(this->get_logger(), "Error seeking to the end of WAV file: %s", sf_strerror(file));
        sf_close(file);
        return;
    }

    std::vector<int32_t> filteredBuffer(buffer);
    applyLowPassFilter(filteredBuffer);

    // Write the new frames to the file
    count = sf_write_int(file, filteredBuffer.data(), frames * sfinfo.channels);
    if(count != frames * sfinfo.channels) {
        RCLCPP_ERROR(this->get_logger(), "Error writing to WAV file: %s", sf_strerror(file));
    }

    sf_close(file);
}

void MicNode::startRecording(const std::shared_ptr<rae_msgs::srv::RecordAudio::Request> request,
                             const std::shared_ptr<rae_msgs::srv::RecordAudio::Response> response) {
    // Start recording when the service is called
    recording_ = true;
    wav_filename_ = request->file_location;
    if(stop_timer_) {
        stop_timer_->reset();
    } else {
        // Create a new timer if it doesn't exist
        stop_timer_ = this->create_wall_timer(std::chrono::seconds(30), std::bind(&MicNode::timeoutRecording, this));
    }

    response->success = true;
    response->message = "Recording started. File will be saved to " + wav_filename_ + ".";
    RCLCPP_INFO(get_logger(), "Recording started.");
}

void MicNode::stopRecording(const std::shared_ptr<rae_msgs::srv::StopRecording::Request> stop_request,
                            const std::shared_ptr<rae_msgs::srv::StopRecording::Response> stop_response) {
    recording_ = false;
    stop_timer_->cancel();  // Stop the timer
    stop_response->success = true;
    stop_response->message = "Recording started. File will be saved to " + wav_filename_ + ".";
    RCLCPP_INFO(get_logger(), "Recording stopped.");
}

void MicNode::timeoutRecording() {
    recording_ = false;
    stop_timer_->cancel();  // Stop the timer
    RCLCPP_INFO(get_logger(), "Recording stopped. Timeout reached.");
}

};  // namespace rae_hw

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rae_hw::MicNode>(rclcpp::NodeOptions());
    rclcpp::executors::StaticSingleThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.spin();
    rclcpp::shutdown();

    return 0;
}