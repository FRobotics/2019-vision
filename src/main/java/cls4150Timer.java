// ===============================================================================
//      class F4150Timer
//          generic elapsed time timer
//
//      use System.nanoTime because it is NOT sensetive to wall clock time.
//
//      isDone will return true if timer has not been started.
//
// ----------------------- revision history --------------------------------------
//
// ===============================================================================
public class cls4150Timer {


    public cls4150Timer() {
        return;
    }

    public cls4150Timer( double IntervalSecs ) {
        
        // -------- set interval seconds
        this.setInterval(IntervalSecs);
        return;
    }

    // ---------private class data
    private boolean boolStarted = false;
    private double dblIntervalSecs = -1.0D;
    private double dblRemainingSecs = 0.0D;
    private long lngIntervalSecs = 0L;
    private long lngEndTime = 0L;

    // --------------------------------------------------------------------------
    public void setInterval( double IntervalSecs ) {
        if ( IntervalSecs >= 0.0D ) {
            dblIntervalSecs = IntervalSecs;
            lngIntervalSecs = (long) ( dblIntervalSecs * 1.0E+9D);
        }
        return;
    }

    // --------------------------------------------------------------------------
    public void startTimer() {

        if ( lngIntervalSecs >= 0L) {
            lngEndTime = System.nanoTime() + lngIntervalSecs;
            boolStarted = true;
            dblRemainingSecs = dblIntervalSecs;
        }
        return;
    }

    // --------------------------------------------------------------------------
    public boolean isDone() {

        long lngCurrent;

        lngCurrent = System.nanoTime();
        dblRemainingSecs = 1.0E-9D * (double) Math.max( 0L, lngEndTime - lngCurrent ); 

        return !boolStarted || ( dblRemainingSecs <= 0.0D );

    }

}

