import sys, os, time, signal, multiprocessing
from optparse import OptionParser

import gi
gi.require_version("Gst", '1.0')
from gi.repository import Gst, GLib
Gst.init(None)

class GSTPipeline(object):
    def __init__(self):
        self.pipeline = None
        self.running = False
        self.error_handler = None
        self.eos_handler = None

    def set_error_handler(self, fn):
        self.error_handler = fn

    def set_eos_handler(self, fn):
        self.eos_handler = fn

    def source(self):
        return self.pipeline.get_by_name('source')

    def tee(self):
        return self.pipeline.get_by_name('tee')

    def sink(self):
        return self.pipeline.get_by_name('sink')

    def set_location(self, uri):
        self.rtmpsink().set_property('location', uri)

    def start(self):
        if not self.running:
            self.running = True
            bus = self.pipeline.get_bus()
            bus.enable_sync_message_emission()
            bus.connect("sync-message", self.on_bus_message, self)
            self.pipeline.set_state(Gst.State.PLAYING)

    def stop(self):
        if self.running:
            bus = self.pipeline.get_bus()
            bus.disable_sync_message_emission()
            self.pipeline.set_state(Gst.State.NULL)
            self.running = False

    def on_bus_message(self, bus, message, userdata):
        if message.type == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            print("Error received from element %s: %s" % (message.src.get_name(), err))
            print("Debug information %s" % debug)
            self.handle_error()
        elif message.type == Gst.MessageType.EOS:
            print("End-of-stream")
            self.handle_eos()
        elif message.type == Gst.MessageType.STATE_CHANGED:
            if isinstance(message.src, Gst.Pipeline):
                old_state, new_state, pending_state = message.parse_state_changed()
                print("Pipeline state changed from %s to %s." % (old_state.value_nick, new_state.value_nick))
        #else:
        #    print("Unexpected message received.")

        return True

    def handle_error(self):
        r = False
        if self.error_handler is not None:
            r = self.error_handler()
        if not r:
            self.stop()

    def handle_eos(self):
        r = False
        if self.eos_handler is not None:
            r = self.eos_handler()
        if not r:
            self.stop()

    def main_loop(self, ev_exit):
        bus = self.pipeline.get_bus()
        timeout_ns = 10000000

        # start pipeline
        self.pipeline.set_state(Gst.State.PLAYING)

        while not ev_exit.is_set():
            message = bus.timed_pop_filtered(timeout_ns, Gst.MessageType.ANY)
            if message:
                if message.type == Gst.MessageType.ERROR:
                    err, debug = message.parse_error()
                    print("Error received from element %s: %s" % (
                        message.src.get_name(), err))
                    print("Debugging information: %s" % debug)
                    break
                elif message.type == Gst.MessageType.WARNING:
                    warning, debug = message.parse_warning()
                    print(warning, debug)
                elif message.type == Gst.MessageType.EOS:
                    print("End-Of-Stream reached.")
                    break
                elif message.type == Gst.MessageType.STATE_CHANGED:
                    if isinstance(message.src, Gst.Pipeline):
                        old_state, new_state, pending_state = message.parse_state_changed()
                        print("Pipeline state changed from %s to %s." %
                              (old_state.value_nick, new_state.value_nick))
                else:
                    print("Unexpected message received.")

        # Stop pipline
        self.pipeline.set_state(Gst.State.NULL)

    @staticmethod
    def from_uri(uri):
        if uri.startswith('videotestsrc'):
            pipeline = VideotestPipeline(uri)
        elif uri.startswith('rtsp://'):
            pipeline = RTSPPipeline(uri)
        else:
            return None

        return pipeline


class VideotestPipeline(GSTPipeline):
    def __init__(self, uri):
        super(VideotestPipeline, self).__init__()
        if uri.startswith('videotestsrc://'):
            pattern = uri[15:]
        if pattern is None or pattern == '':
            pattern = 'smpte'
        self.pipeline = Gst.parse_launch(
            'videotestsrc pattern={} name=source ! x264enc tune=zerolatency rc-lookahead=5 ! flvmux streamable=true ! rtmpsink name=sink'.format(pattern)
        )


class RTSPPipeline(GSTPipeline):
    def __init__(self, uri):
        super(RTSPPipeline, self).__init__()
        self.pipeline = Gst.parse_launch(
            'rtspsrc name=source location={} latency=0 udp-reconnect=1 timeout=5000000 ! h264parse ! rtph264depay ! flvmux streamable=true ! rtmpsink sync=false name=sink'.format(uri)
        )


class RtmpHandler(object):
    def __init__(self, uri):
        self.pipeline = GSTPipeline.from_uri(uri)

    def start(self, uri):
        self.pipeline.stop()

        sink = self.pipeline.sink()
        sink.set_property('location', uri)

        self.pipeline.start()

    def stop(self):
        self.pipeline.stop()

    def update(self):
        if self.pipeline.running:
            # check pipeline status
            pass


class Application(object):
    def __init__(self, optargs):
        (opts, args) = optargs

        if opts.stream_url is None or not opts.stream_url.startswith("rtmp://"):
            print("Unsupported stream %s" % opts.stream_url)
            sys.exit(1)

        self.ev_exit = multiprocessing.Event()

        signal.signal(signal.SIGINT, self.exit_gracefully)
        signal.signal(signal.SIGTERM, self.exit_gracefully)

        rtmpHandler = RtmpHandler(opts.input)
        rtmpHandler.start(opts.stream_url)

        while not self.ev_exit.is_set():
            time.sleep(1)

        rtmpHandler.stop()

    def exit_gracefully(self, signum, frame):
        print("exit_gracefully\n")
        self.ev_exit.set()


def main():
    parser = OptionParser('wahaha [options]')
    parser.add_option("--input", dest="input", type="str",
                      help="Live Stream URL", default="videotestsrc")
    parser.add_option("--stream-url", dest="stream_url", type="str",
                      help="Stream URL", default=None)
    optsargs = parser.parse_args()
    (opts, args) = optsargs
    Application(optsargs)


if __name__ == '__main__':
    main()
