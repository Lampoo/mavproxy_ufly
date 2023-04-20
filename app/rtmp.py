import sys, os, time, signal, multiprocessing
from threading import Thread
from optparse import OptionParser

import gi
gi.require_version("Gst", '1.0')
from gi.repository import Gst, GLib
Gst.init(None)

class GSTPipeline(Thread):
    def __init__(self):
        super(GSTPipeline, self).__init__()
        self.pipeline = None
        self.time_to_quit = False

    def source(self):
        return self.pipeline.get_by_name('source')

    def tee(self):
        return self.pipeline.get_by_name('tee')

    def sink(self):
        return self.pipeline.get_by_name('sink')

    def set_location(self, uri):
        self.sink().set_property('location', uri)

    def run(self):
        delay = 0
        while not self.time_to_quit:
            if delay > 0:
                time.sleep(delay)
            self.pipeline.set_state(Gst.State.PLAYING)
            delay = self.process_message()
            self.pipeline.set_state(Gst.State.NULL)

    def process_message(self, timeout_ns = Gst.CLOCK_TIME_NONE):
        bus = self.pipeline.get_bus()
        while True:
            message = bus.timed_pop_filtered(timeout_ns, Gst.MessageType.ANY)
            if message:
                if message.type == Gst.MessageType.ERROR:
                    err, debug = message.parse_error()
                    print("Error received from element %s: %s" % (
                        message.src.get_name(), err))
                    print("Debugging information: %s" % debug)
                    return 1
                elif message.type == Gst.MessageType.EOS:
                    print("End-Of-Stream reached.")
                    return 1
                elif message.type == Gst.MessageType.WARNING:
                    warning, debug = message.parse_warning()
                    print(warning, debug)
                elif message.type == Gst.MessageType.STATE_CHANGED:
                    if isinstance(message.src, Gst.Pipeline):
                        old_state, new_state, pending_state = message.parse_state_changed()
                        print("Pipeline state changed from %s to %s." %
                            (old_state.value_nick, new_state.value_nick))
            else:
                # Timeout
                return 0

    def stop(self):
        self.time_to_quit = True
        self.pipeline.send_event(Gst.Event.new_eos())

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
        else:
            pattern = ''
        if pattern is None or pattern == '':
            pattern = 'smpte'
        self.pipeline = Gst.parse_launch(
            'videotestsrc pattern={} name=source is-live=true ! tee name=t ! x264enc tune=zerolatency rc-lookahead=5 ! flvmux streamable=true ! rtmpsink sync=false name=sink'.format(pattern)
        )


class RTSPPipeline(GSTPipeline):
    def __init__(self, uri):
        super(RTSPPipeline, self).__init__()
        self.pipeline = Gst.parse_launch(
            'rtspsrc name=source location={} latency=0 udp-reconnect=1 timeout=5000000 ! rtph264depay ! flvmux streamable=true ! rtmpsink sync=false name=sink'.format(uri)
        )


class Application(object):
    def __init__(self, optargs):
        (opts, args) = optargs

        if opts.stream_url is None or not opts.stream_url.startswith("rtmp://"):
            print("Unsupported stream %s" % opts.stream_url)
            sys.exit(1)

        self.ev_exit = multiprocessing.Event()

        signal.signal(signal.SIGINT, self.exit_gracefully)
        signal.signal(signal.SIGTERM, self.exit_gracefully)

        pipeline = GSTPipeline.from_uri(opts.input)
        pipeline.set_location(opts.stream_url)
        pipeline.start()

        while not self.ev_exit.is_set():
            time.sleep(1)

        pipeline.stop()
        pipeline.join()

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
