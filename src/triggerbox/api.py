class TriggerboxAPI:

    #Callbacks from the underlying hardware
    framerate_callback = None   #func(expected_trigger_rate)
    clockmodel_callback = None  #func(gain, offset)
    fatal_error_callback = None #func(msg)
    connected_callback = None   #func(name, device)

    def _api_callback(self, cb_obj, *args):
        if cb_obj is not None:
            cb_obj(*args)

    #ClientAPI
    def have_estimate(self):
        raise NotImplementedError

    def wait_for_estimate(self):
        raise NotImplementedError

    def timestamp2framestamp(self, timestamp ):
        raise NotImplementedError

    def framestamp2timestamp(self, framestamp ):
        raise NotImplementedError

    def get_frames_per_second(self,wait_for_valid=True):
        raise NotImplementedError

    def set_frames_per_second(self,value):
        raise NotImplementedError

    def set_frames_per_second_blocking(self, *args, **kwargs):
        raise NotImplementedError

    def synchronize(self, pause_duration_seconds=2 ):
        raise NotImplementedError


