import numpy as np

def model_remote_to_local(remote_timestamps, local_timestamps):
    """for timestamps"""
    a1=remote_timestamps[:,np.newaxis]
    a2=np.ones( (len(remote_timestamps),1))
    A = np.hstack(( a1,a2))
    b = local_timestamps[:,np.newaxis]
    x,resids,rank,s = np.linalg.lstsq(A,b)
    gain = x[0,0]
    offset = x[1,0]
    return gain, offset, resids

class TimeModel:
    def __init__(self,gain,offset):
        self.gain = gain
        self.offset = offset
    def timestamp2framestamp(self, mainbain_timestamp ):
        return (mainbain_timestamp-self.offset)/self.gain
    def framestamp2timestamp(self, framestamp ):
        return framestamp*self.gain + self.offset

class TimeFitError(ValueError):
    pass

def get_time_model(T,C, max_residual=5e-5):
    # fit linear model of relationship mainbrain timestamp and usb trigger_device framestamp
    gain, offset, resid = model_remote_to_local( T, C)

    if not len(resid):
        raise TimeFitError('insufficient data for fit')
    if resid[0] > max_residual:
        raise TimeFitError('fit is poor quality (residuals = %s)'%resid[0])
    time_model = TimeModel(gain, offset)
    return time_model
