/* EspRangeTest — service worker.
 *
 * Why this exists: Web Bluetooth, geolocation and the screen wake lock all
 * require a secure context, which in practice means loading this page over
 * https. A range walk is exactly the situation where there is no signal to
 * load it with — that is the thing being measured. Installing the GitHub Pages
 * copy as a PWA keeps the https:// origin, and everything it unlocks, working
 * with no network at all.
 *
 * Strategy
 *   precache            the whole app on install. It is one self-contained
 *                       index.html plus icons, so this is the entire thing.
 *   same-origin GETs    stale-while-revalidate — serve from cache instantly,
 *                       refresh in the background, tell the page if the copy
 *                       on the server changed so it can offer a reload.
 *                       Navigations resolve to the cached index.html under the
 *                       same rule. Deliberately not network-first: out in a
 *                       field there may be a bar of signal that resolves DNS
 *                       and then stalls, and the app would stall with it.
 *   everything else     passthrough, untouched.
 *
 * Note on sharing an origin: bucketshoes.github.io also hosts the Moonshot
 * Avionics dashboard under its own path, with its own service worker. Caches
 * and service-worker registrations are origin-wide, so everything here is
 * namespaced by CACHE_PREFIX and by scope — this worker must never delete a
 * cache or unregister a worker that belongs to the other app.
 *
 * Because of stale-while-revalidate you do NOT normally need to bump
 * CACHE_VERSION when you edit index.html: deploy, load the page once (with
 * internet), and the page offers to reload into the new build. Bump it only to
 * force-evict every client, e.g. after removing a precached file.
 */

var CACHE_PREFIX  = 'esprangetest-';
var CACHE_VERSION = 'v1';
var CACHE_NAME    = CACHE_PREFIX + CACHE_VERSION;

// Everything required to boot with zero network. Relative to this file, so the
// app can move to a different path or domain untouched.
var PRECACHE = [
  'index.html',
  'manifest.webmanifest',
  'icons/icon-192.png',
  'icons/icon-512.png',
  'icons/icon-192-maskable.png',
  'icons/icon-512-maskable.png',
  'icons/apple-touch-icon.png',
  'icons/favicon-32.png'
];

// The files worth re-checking on every launch. A reload does not reliably
// route subresources through this worker — Chrome can answer them from the
// renderer's memory cache — so passive revalidation alone would miss deploys.
// The app is one file; the icons change about never.
var WATCH = ['index.html'];

// ---------------------------------------------------------------- install --

self.addEventListener('install', function(ev) {
  ev.waitUntil(
    caches.open(CACHE_NAME).then(function(cache) {
      // cache:'reload' bypasses the HTTP cache so an install always pulls the
      // build that is actually on the server, not GitHub Pages' 10-minute copy.
      return Promise.all(PRECACHE.map(function(url) {
        return fetch(new Request(url, { cache: 'reload' })).then(function(res) {
          if (!res.ok) throw new Error(res.status + ' ' + url);
          return cache.put(url, res);
        }).catch(function(err) {
          // One missing icon must not abort the install and leave the user
          // with no offline copy at all.
          console.warn('[sw] precache failed:', url, err.message);
        });
      }));
    })
  );
});

// --------------------------------------------------------------- activate --

self.addEventListener('activate', function(ev) {
  ev.waitUntil(
    caches.keys().then(function(names) {
      return Promise.all(names.map(function(n) {
        // Only ever this app's own old caches — see the origin note above.
        if (n !== CACHE_NAME && n.indexOf(CACHE_PREFIX) === 0) return caches.delete(n);
      }));
    }).then(function() {
      return self.clients.claim();
    })
  );
});

// URLs whose bytes changed on the server since the current page was served.
// Kept here as well as pushed to clients because a background revalidate often
// finishes before the page has attached its message listener — the page asks
// for the backlog with 'check-updates' once it is ready. Cleared on every
// navigation, since by then the page is loading the refreshed files.
var pendingUpdates = [];

// Set by a 'purge' message. Once purging, this worker must not write to the
// cache again: the page is about to delete it, and an in-flight revalidate
// landing its cache.put() afterwards would silently recreate it.
var purging = false;

self.addEventListener('message', function(ev) {
  // Sent after the user accepts the update prompt.
  if (ev.data === 'skip-waiting') { self.skipWaiting(); return; }

  // Sent by the ?nosw recovery hatch. The worker deletes its own caches,
  // because only it knows when its own writes have stopped. Scoped to this
  // app's prefix — the other app on this origin keeps its offline copy.
  if (ev.data === 'purge') {
    purging = true;
    ev.waitUntil(caches.keys().then(function(names) {
      return Promise.all(names.filter(function(n) {
        return n.indexOf(CACHE_PREFIX) === 0;
      }).map(function(n) { return caches.delete(n); }));
    }).then(function() {
      if (ev.source) ev.source.postMessage({ type: 'purged' });
    }));
    return;
  }

  if (ev.data === 'check-updates') {
    if (pendingUpdates.length && ev.source) {
      ev.source.postMessage({ type: 'asset-updated', url: pendingUpdates[0] });
      return;
    }
    // Otherwise go and look. Conditional requests, so this is a few hundred
    // bytes of 304s when nothing changed, and nothing at all when offline.
    ev.waitUntil(caches.open(CACHE_NAME).then(function(cache) {
      return Promise.all(WATCH.map(function(name) {
        var url = new URL(name, self.location.href).href;
        return cache.match(url).then(function(cached) {
          if (!cached) return null;
          return revalidate(cache, url, cached);
        });
      }));
    }));
  }
});

// ------------------------------------------------------------------ fetch --

function notifyClients(msg) {
  if (msg.type === 'asset-updated' && pendingUpdates.indexOf(msg.url) < 0) {
    pendingUpdates.push(msg.url);
  }
  self.clients.matchAll({ type: 'window' }).then(function(cs) {
    cs.forEach(function(c) { c.postMessage(msg); });
  });
}

// Cheap change detection: GitHub Pages sends a strong ETag. Fall back to
// Last-Modified, then to "assume unchanged" rather than nagging every load.
function stamp(res) {
  if (!res) return null;
  return res.headers.get('ETag') || res.headers.get('Last-Modified') || null;
}

// Re-fetch one URL, update the cache, and tell open pages if the bytes on the
// server actually changed. Resolves to null when offline.
function revalidate(cache, url, cached) {
  // no-cache forces a conditional request, so an edit deployed minutes ago is
  // not hidden behind GitHub Pages' max-age.
  if (purging) return Promise.resolve(null);
  return fetch(new Request(url, { cache: 'no-cache' })).then(function(res) {
    if (!res || !res.ok || purging) return null;
    var before = stamp(cached), after = stamp(res);
    return cache.put(url, res.clone()).then(function() {
      if (cached && before && after && before !== after) {
        notifyClients({ type: 'asset-updated', url: url });
      }
      return res;
    });
  }).catch(function() {
    return null;   // offline: the cached copy is the answer
  });
}

// Stale-while-revalidate for one URL.
//
// ev.waitUntil() has to be called synchronously from the fetch handler — do it
// from inside a .then() and the event may already have finished, which throws
// and silently kills the background refresh. So the revalidate promise is
// built here and handed to waitUntil straight away.
function staleWhileRevalidate(ev, url) {
  var cacheP = caches.open(CACHE_NAME);
  // ignoreSearch so a cache-busted URL still hits its precached entry. Older
  // copies of the page (and any tab still running one) request
  // `app.js?_=<timestamp>`, a URL that can never be in the cache; without this
  // they get a 504 offline, no script and no stylesheet load, and the page
  // renders as dead unstyled HTML that looks like a broken app rather than a
  // stale one.
  var cachedP = cacheP.then(function(cache) {
    return cache.match(url, { ignoreSearch: true });
  });
  var freshP = Promise.all([cacheP, cachedP]).then(function(r) {
    return revalidate(r[0], url, r[1]);
  });

  ev.waitUntil(freshP);

  ev.respondWith(cachedP.then(function(cached) {
    if (cached) return cached;                    // instant, works offline
    return freshP.then(function(res) {
      return res || new Response('offline and not cached', {
        status: 504, statusText: 'Offline',
        headers: { 'Content-Type': 'text/plain' }
      });
    });
  }));
}

self.addEventListener('fetch', function(ev) {
  var req = ev.request;
  if (req.method !== 'GET') return;

  // Mid-purge the cache is being torn down, so stop intercepting and let
  // requests go straight to the network rather than answering 504 from a
  // cache that no longer exists.
  if (purging) return;

  var url;
  try { url = new URL(req.url); } catch (e) { return; }

  if (url.origin !== self.location.origin) return;

  // Only handle files under this app's own directory — the other app on this
  // origin has its own worker and must be left alone.
  var scope = new URL('./', self.location.href).pathname;
  if (url.pathname.indexOf(scope) !== 0) return;

  // Navigations always render the cached page, then check for a new one in the
  // background. index.html is normalised to a single cache key so that "/",
  // "/index.html" and the PWA start_url all share one entry.
  if (req.mode === 'navigate') {
    pendingUpdates.length = 0;
    staleWhileRevalidate(ev, new URL('index.html', self.location.href).href);
    return;
  }

  staleWhileRevalidate(ev, req.url);
});
