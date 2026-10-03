// Shared by the controller and SD pages. History stays in this browser.
window.sequenceHistory = (() => {
    const key = 'lpov-recent-sequences', selectedKey = 'lpov-selected-sequence', limit = 12;
    let paths = [], selected = '';
    const valid = path => typeof path === 'string' && path.startsWith('/') && /\.fseq$/i.test(path);
    function load() {
        try {
            const saved = JSON.parse(localStorage.getItem(key) || '[]');
            paths = Array.isArray(saved) ? [...new Set(saved.filter(valid))].slice(0, limit) : [];
            selected = localStorage.getItem(selectedKey) || '';
            if (!valid(selected)) selected = '';
        } catch {}
    }
    function save() {
        try {
            localStorage.setItem(key, JSON.stringify(paths));
            localStorage.setItem(selectedKey, selected);
        } catch {}
    }
    function remember(path) {
        if (!valid(path)) return;
        paths = [path, ...paths.filter(p => p !== path)].slice(0, limit);
        selected = path;
        save();
    }
    function forget(path) {
        paths = paths.filter(p => p !== path);
        if (selected === path) selected = '';
        save();
    }
    function reconcile(directory, files) {
        const parent = (directory.replace(/\/$/, '') || '') + '/';
        const present = new Set(files.filter(f => !f.directory).map(f => f.path));
        for (const path of paths.slice()) {
            if (path.slice(0, path.lastIndexOf('/') + 1) === parent && !present.has(path)) forget(path);
        }
    }
    load();
    addEventListener('storage', event => {
        if (event.key === key || event.key === selectedKey || event.key === null) {
            load();
            dispatchEvent(new Event('sequencehistorychange'));
        }
    });
    return {list: () => paths.slice(), selected: () => selected, remember, forget, reconcile};
})();
