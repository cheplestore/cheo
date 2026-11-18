# "Hey Cheo" wake word model

Put the Edge Impulse **Arduino library .zip** here (only one .zip at a time).
GitHub then builds it into Cheo's firmware automatically, and Cheo updates himself.

## Steps
1. **Record** on Cheo's page: `cheo.local/train` (needs your Edge Impulse API key on the Settings page).
   About 100 "Hey Cheo", 50 noise, 50 other words.
2. **Edge Impulse** (project `cheo-wake`):
   - **Dashboard**: scroll down, click **Perform train / test split**.
   - **Create impulse**: window size **1000 ms**, window increase **250 ms**, frequency **16000 Hz**.
     Add processing block **Audio (MFCC)**, learning block **Classification**. Save.
   - **MFCC**: Save parameters, then **Generate features**.
   - **Classifier**: **Start training** (default settings are fine). Aim for 90%+ accuracy.
   - **Model testing**: Classify all. Check "hey_cheo" is recognized.
   - **Deployment**: choose **Arduino library**, click **Build**. A .zip downloads.
3. **Upload the .zip into this `wake/` folder** on GitHub (Add file -> Upload files) and commit.

To retrain later: record more on `/train`, retrain in Edge Impulse, and replace the .zip here.

## Model history
- 1.0.1: unknown 40% right, 20% of other talk mistaken for "Hey Cheo" (false wake-ups)
- 1.0.2: retrained with many more unknown/noise clips: accuracy 86.5%, unknown 82.6%, talk mistaken for "Hey Cheo" 8.7%
